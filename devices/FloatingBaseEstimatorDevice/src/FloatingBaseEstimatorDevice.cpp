/**
 * @file FloatingBaseEstimatorDevice.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/FloatingBaseEstimatorDevice.h>
#include <BipedalLocomotion/YarpUtilities/Helper.h>

#include <BipedalLocomotion/FloatingBaseEstimators/InvariantEKFBaseEstimator.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/YARPConversions.h>
#include <yarp/os/LogStream.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion::ParametersHandler;

FloatingBaseEstimatorDevice::FloatingBaseEstimatorDevice(double period,
                                                         yarp::os::ShouldUseSystemClock useSystemClock)
        : yarp::os::PeriodicThread(period, useSystemClock)
{
}

FloatingBaseEstimatorDevice::FloatingBaseEstimatorDevice()
        : yarp::os::PeriodicThread(0.01, yarp::os::ShouldUseSystemClock::No)
{
}

FloatingBaseEstimatorDevice::~FloatingBaseEstimatorDevice()
{
}

bool FloatingBaseEstimatorDevice::open(yarp::os::Searchable& config)
{
    YarpUtilities::getElementFromSearchable(config, "robot", m_robot);
    YarpUtilities::getElementFromSearchable(config, "port_prefix", m_portPrefix);
    YarpUtilities::getElementFromSearchable(config, "base_link_imu", m_baseLinkImuName);
    YarpUtilities::getElementFromSearchable(config, "left_foot_front_wrench", m_leftFootWrenchNames.front);
    YarpUtilities::getElementFromSearchable(config, "left_foot_rear_wrench", m_leftFootWrenchNames.rear);
    YarpUtilities::getElementFromSearchable(config, "right_foot_front_wrench", m_rightFootWrenchNames.front);
    YarpUtilities::getElementFromSearchable(config, "right_foot_rear_wrench", m_rightFootWrenchNames.rear);
    double devicePeriod{0.01};

    if (YarpUtilities::getElementFromSearchable(config, "sampling_period_in_s", devicePeriod))
    {
        setPeriod(devicePeriod);
    }

    if (!setupRobotModel(config))
    {
        return false;
    }

    if (!setupRobotSensorBridge(config))
    {
        return false;
    }

    if (!setupFeetContactStateMachines(config))
    {
        return false;
    }

    if (!setupBaseEstimator(config))
    {
        return false;
    }

    if (!YarpUtilities::getElementFromSearchable(config, "publish_rostf", m_publishROSTF))
     {
         m_publishROSTF = false;
     }

     if (m_publishROSTF)
     {
         if (!loadTransformBroadcaster())
         {
             return false;
         }
     }

    return true;
}

bool FloatingBaseEstimatorDevice::setupRobotModel(yarp::os::Searchable& config)
{
    constexpr auto logPrefix = "[FloatingBaseEstimatorDevice::setupRobotModel]";

    std::string modelFileName;
    std::vector<std::string> jointsList;
    if (!YarpUtilities::getElementFromSearchable(config, "model_file", modelFileName))
    {
        log()->error("{} Missing required parameter \"model_file\"", logPrefix);
        return false;
    }

    if (!YarpUtilities::getVectorFromSearchable(config, "joint_list", jointsList))
    {
        log()->error("{} Missing required parameter \"joint_list\"", logPrefix);
        return false;
    }

    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();
    std::string modelFilePath{rf.findFileByName(modelFileName)};
    log()->info("{} Loading model from {}", logPrefix, modelFilePath);

    iDynTree::ModelLoader modelLoader;
    if (!modelLoader.loadReducedModelFromFile(modelFilePath, jointsList))
    {
        log()->error("{} Could not load robot model", logPrefix);
        return false;
    }

    m_model = modelLoader.model();
    return true;
}

bool FloatingBaseEstimatorDevice::setupRobotSensorBridge(yarp::os::Searchable& config)
{
    constexpr auto logPrefix = "[FloatingBaseEstimatorDevice::setupRobotSensorBridge]";

    auto bridgeConfig = config.findGroup("RobotSensorBridge");
    if (bridgeConfig.isNull())
    {
        log()->error("{} Missing required group \"RobotSensorBridge\"", logPrefix);
        return false;
    }

    std::shared_ptr<YarpImplementation> bridgeHandler = std::make_shared<YarpImplementation>();
    bridgeHandler->set(bridgeConfig);

    m_robotSensorBridge = std::make_unique<YarpSensorBridge>();
    if (!m_robotSensorBridge->initialize(bridgeHandler))
    {
        log()->error("{} Could not configure RobotSensorBridge", logPrefix);
        return false;
    }

    return true;
}

bool FloatingBaseEstimatorDevice::setupFeetContactStateMachines(yarp::os::Searchable& config)
{
    constexpr auto logPrefix = "[FloatingBaseEstimatorDevice::setupFeetContactStateMachines]";

    auto csmConfig = config.findGroup("ContactSchmittTrigger");
    if (csmConfig.isNull())
    {
        log()->error("{} Missing required group \"ContactSchmittTrigger\"");
        return false;
    }

    iDynTree::SchmittParams lParams, rParams;
    auto lCSMConfig = csmConfig.findGroup("left_foot");
    if (lCSMConfig.isNull())
    {
        log()->error("{} Could not load left foot contact Schmitt trigger configuration group.", logPrefix);
        return false;
    }
    if (!parseFootSchmittParams(lCSMConfig, lParams))
    {
        log()->error("{} Could not load left foot contact Schmitt trigger parameters", logPrefix);
        return false;
    }

    auto rCSMConfig = csmConfig.findGroup("right_foot");
    if (rCSMConfig.isNull())
    {
        log()->error("{} Could not load right foot contact Schmitt trigger configuration group.", logPrefix);
        return false;
    }
    if (!parseFootSchmittParams(rCSMConfig, rParams))
    {
        log()->error("{} Could not load right foot contact Schmitt trigger parameters", logPrefix);
        return false;
    }

    m_leftFootCSM = std::make_unique<iDynTree::ContactStateMachine>(lParams);
    m_rightFootCSM = std::make_unique<iDynTree::ContactStateMachine>(rParams);

    return true;
}

bool FloatingBaseEstimatorDevice::parseFootSchmittParams(yarp::os::Searchable& config,
                                                         iDynTree::SchmittParams& params)
{
    if (!YarpUtilities::getElementFromSearchable(config, "schmitt_stable_contact_make_time", params.stableTimeContactMake)) { return false; }
    if (!YarpUtilities::getElementFromSearchable(config, "schmitt_stable_contact_break_time", params.stableTimeContactBreak)) { return false; }
    if (!YarpUtilities::getElementFromSearchable(config, "schmitt_contact_make_force_threshold", params.contactMakeForceThreshold)) { return false; }
    if (!YarpUtilities::getElementFromSearchable(config, "schmitt_contact_break_force_threshold", params.contactBreakForceThreshold)) { return false; }

    return true;
}

bool FloatingBaseEstimatorDevice::setupBaseEstimator(yarp::os::Searchable& config)
{
    constexpr auto logPrefix = "[FloatingBaseEstimatorDevice::setupBaseEstimator]";

    if (!YarpUtilities::getElementFromSearchable(config, "estimator_type", m_estimatorType))
    {
        log()->error("{} Missing required parameter \"estimator_type\"", logPrefix);
        return false;
    }

    if (m_estimatorType == "InvEKF")
    {
        m_estimator = std::make_unique<InvariantEKFBaseEstimator>();
    }

    std::shared_ptr<YarpImplementation> originalHandler = std::make_shared<YarpImplementation>();
    originalHandler->set(config);
    IParametersHandler::shared_ptr parameterHandler = originalHandler;

    m_kinDyn = std::make_shared<iDynTree::KinDynComputations>();
    m_kinDyn->loadRobotModel(m_model);
    if (!m_estimator->initialize(parameterHandler, m_kinDyn))
    {
        log()->error("{} Could not configure estimator", logPrefix);
        return false;
    }
    return true;
}

bool FloatingBaseEstimatorDevice::attachAll(const yarp::dev::PolyDriverList & poly)
{
    constexpr auto logPrefix = "[FloatingBaseEstimatorDevice::attachAll]";

    if (!m_robotSensorBridge->setDriversList(poly))
    {
        log()->error("{} Failed to attach to devices through RobotSensorBridge.", logPrefix);
        return false;
    }

    if (!openCommunications())
    {
        log()->error("{} Could not open ports for publishing outputs.", logPrefix);
        return false;
    }

    start();
    return true;
}

bool FloatingBaseEstimatorDevice::openCommunications()
{
    if (!openBufferedSigPort(m_comms.floatingBaseStatePort, m_portPrefix, "/floating_base/state:o"))
    {
        return false;
    }

    openBufferedSigPort(m_comms.internalStateAndStdDevPort, m_portPrefix, "/internal_state/stateAndStdDev:o");
    openBufferedSigPort(m_comms.contactStatePort, m_portPrefix, "/contacts/stateAndNormalForce:o");
    return true;
}

bool FloatingBaseEstimatorDevice::openBufferedSigPort(yarp::os::BufferedPort<yarp::sig::Vector>& port,
                                                      const std::string& portPrefix,
                                                      const std::string& address)
{
    constexpr auto logPrefix = "[FloatingBaseEstimatorDevice::openBufferedSigPort]";
    bool ok{false};
    ok = port.open(portPrefix + address);
    if (!ok)
    {
        log()->error("{} error opening port {}.", logPrefix, portPrefix + address);
        return false;
    }
    return true;
}

void FloatingBaseEstimatorDevice::run()
{
    constexpr auto logPrefix = "[FloatingBaseEstimatorDevice::run]";

    // advance sensor bridge
    if (!m_robotSensorBridge->advance())
    {
        log()->warn("{} Advance Sensor bridge failed.", logPrefix);
        return;
    }

    // update estimator measurements
    if (!updateMeasurements())
    {
        log()->warn("{} Measurement updates failed.", logPrefix);
        return;
    }

    // advance estimator
    if (!m_estimator->advance())
    {
        log()->warn("{} Advance estimator failed.", logPrefix);
        return;
    }

    publish();
}

bool FloatingBaseEstimatorDevice::updateMeasurements()
{
    bool ok{true};

    ok = ok && updateContactStates();
    ok = ok && updateInertialBuffers();
    ok = ok && updateKinematics();

    return ok;
}

bool FloatingBaseEstimatorDevice::updateInertialBuffers()
{
    Eigen::Matrix<double, 12, 1> imuMeasure;
    if (!m_robotSensorBridge->getIMUMeasurement(m_baseLinkImuName, imuMeasure))
    {
        return false;
    }

    const int accOffset{3};
    const int gyroOffset{6};
    if (!m_estimator->setIMUMeasurement(imuMeasure.segment<3>(accOffset), imuMeasure.segment<3>(gyroOffset)))
    {
        return false;
    }

    return true;
}

bool FloatingBaseEstimatorDevice::updateContactStates()
{
    BipedalLocomotion::Math::Wrenchd leftFootFrontWrench, leftFootRearWrench, rightFootFrontWrench, rightFootRearWrench;
    double lfTimeStamp, rfTimeStamp;
    bool ok{true};
    ok = ok && m_robotSensorBridge->getCartesianWrench(m_leftFootWrenchNames.front, leftFootFrontWrench, lfTimeStamp);
    ok = ok && m_robotSensorBridge->getCartesianWrench(m_leftFootWrenchNames.rear, leftFootRearWrench, lfTimeStamp);
    if (ok)
    {
        m_currentLeftContactNormal = leftFootFrontWrench.force().z() + leftFootRearWrench.force().z();
        m_leftFootCSM->contactMeasurementUpdate(lfTimeStamp, m_currentLeftContactNormal); // fz
    }

    ok = ok && m_robotSensorBridge->getCartesianWrench(m_rightFootWrenchNames.front, rightFootFrontWrench, rfTimeStamp);
    ok = ok && m_robotSensorBridge->getCartesianWrench(m_rightFootWrenchNames.rear, rightFootRearWrench, rfTimeStamp);

    if (ok)
    {
        m_currentRightContactNormal = rightFootFrontWrench.force().z() + rightFootRearWrench.force().z();
        m_rightFootCSM->contactMeasurementUpdate(rfTimeStamp, m_currentRightContactNormal); // fz
    }

    if (!ok)
    {
        return false;
    }

    m_currentRightFootState = m_leftFootCSM->contactState();
    m_currentRightFootState = m_rightFootCSM->contactState();
    if (!m_estimator->setContacts(m_currentLeftFootState, m_currentRightFootState))
    {
        return false;
    }

    return true;
}

bool FloatingBaseEstimatorDevice::updateKinematics()
{
    Eigen::VectorXd encoders(m_model.getNrOfDOFs()), encoderSpeeds(m_model.getNrOfDOFs());
    double timeStamp;
    if (!m_robotSensorBridge->getJointPositions(encoders))
    {
        return false;
    }

    if (!m_robotSensorBridge->getJointVelocities(encoderSpeeds))
    {
        return false;
    }

    if (!m_estimator->setKinematics(encoders, encoderSpeeds))
    {
        return false;
    }
    return true;
}

void FloatingBaseEstimatorDevice::publish()
{
    if (m_estimator->isOutputValid())
    {
        auto estimatorOut = m_estimator->getOutput();
        publishBaseLinkState(estimatorOut);
        publishInternalStateAndStdDev(estimatorOut);
    }
    publishFootContactStatesAndNormalForces();
}

void FloatingBaseEstimatorDevice::publishBaseLinkState(const FloatingBaseEstimators::Output& estimatorOut)
{
    constexpr auto logPrefix = "[FloatingBaseEstimatorDevice::publishBaseLinkState]";

    size_t stateVecSize{12};
    size_t rpyOffset{3};
    size_t linVelOffset{6};
    size_t angVelOffset{9};

    m_basePos = estimatorOut.basePose.translation();
    m_baseRPY = estimatorOut.basePose.quat().toRotationMatrix().eulerAngles(2, 1, 0).reverse(); // rpy euler angles in xyz convention
    m_baseLinearVel = estimatorOut.baseTwist.head<3>();
    m_baseAngularVel = estimatorOut.baseTwist.tail<3>();

    yarp::sig::Vector& stateVec = m_comms.floatingBaseStatePort.prepare();
    stateVec.clear();
    stateVec.resize(stateVecSize);

    for (size_t idx = 0; idx < rpyOffset; idx++) { stateVec(idx) = m_basePos(idx); }
    for (size_t idx = rpyOffset; idx < linVelOffset; idx++) { stateVec(idx) = m_baseRPY(idx - rpyOffset); }
    for (size_t idx = linVelOffset; idx < angVelOffset; idx++) { stateVec(idx) = m_baseLinearVel(idx - linVelOffset); }
    for (size_t idx = angVelOffset; idx < stateVecSize; idx++) { stateVec(idx) = m_baseAngularVel(idx - angVelOffset); }

    m_comms.floatingBaseStatePort.write();

    yarp::sig::Matrix basePoseYARP;
    size_t dim{4};
    basePoseYARP.resize(dim, dim);
    memcpy(basePoseYARP.data(),estimatorOut.basePose.transform().data(),dim*dim*sizeof(double));

    if (m_publishROSTF && m_transformInterface != nullptr)
    {
        if (!m_transformInterface->setTransform("/world", "/base_link", basePoseYARP))
        {
            log()->error("{} Could not publish measured base pose transform from  primary IMU", logPrefix);
        }
    }
}

void FloatingBaseEstimatorDevice::publishInternalStateAndStdDev(const FloatingBaseEstimators::Output& estimatorOut)
{
    size_t vecSize{27};

    const auto& s = estimatorOut.state;
    const auto& d = estimatorOut.stateStdDev;
    Eigen::Vector3d imuRPY = s.imuOrientation.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
    Eigen::Vector3d rfRPY = s.rContactFrameOrientation.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
    Eigen::Vector3d lfRPY = s.lContactFrameOrientation.toRotationMatrix().eulerAngles(2, 1, 0).reverse();

    Eigen::VectorXd internalstateAndStdDev;
    internalstateAndStdDev.resize(vecSize + vecSize);
    internalstateAndStdDev << s.imuPosition, imuRPY, s.imuLinearVelocity,
                              s.rContactFramePosition, rfRPY,
                              s.lContactFramePosition, lfRPY,
                              s.accelerometerBias, s.gyroscopeBias,
                              d.imuPosition, d.imuOrientation, d.imuLinearVelocity,
                              d.rContactFramePosition, d.rContactFrameOrientation,
                              d.lContactFramePosition, d.lContactFrameOrientation,
                              d.accelerometerBias, d.gyroscopeBias;

    yarp::sig::Vector& stateAndStdDev = m_comms.internalStateAndStdDevPort.prepare();
    stateAndStdDev.clear();
    stateAndStdDev.resize(vecSize + vecSize);

    for (size_t idx = 0; idx < 2*vecSize; idx++)
    {
        stateAndStdDev(idx) = internalstateAndStdDev[idx];
    }

    m_comms.internalStateAndStdDevPort.write();
}

void FloatingBaseEstimatorDevice::publishFootContactStatesAndNormalForces()
{
    yarp::sig::Vector& contactVec = m_comms.contactStatePort.prepare();
    contactVec.clear();

    contactVec.resize(4);
    const int scaling_const_for_visualization{300};
    contactVec(0) = scaling_const_for_visualization*static_cast<int>(m_currentLeftFootState);
    contactVec(1) = scaling_const_for_visualization*static_cast<int>(m_currentRightFootState);
    contactVec(2) = m_currentLeftContactNormal;
    contactVec(3) = m_currentRightContactNormal;
    m_comms.contactStatePort.write();
}

bool FloatingBaseEstimatorDevice::detachAll()
{
    std::lock_guard<std::mutex> guard(m_deviceMutex);
    if (isRunning())
    {
        stop();
    }

    return true;
}

void FloatingBaseEstimatorDevice::closeBufferedSigPort(yarp::os::BufferedPort<yarp::sig::Vector>& port)
{
    if (!port.isClosed())
    {
        port.close();
    }

    if (m_publishROSTF)
    {
        m_transformInterface = nullptr;
    }
}

void FloatingBaseEstimatorDevice::closeCommunications()
{
    closeBufferedSigPort(m_comms.floatingBaseStatePort);
    closeBufferedSigPort(m_comms.internalStateAndStdDevPort);
    closeBufferedSigPort(m_comms.contactStatePort);
}

bool FloatingBaseEstimatorDevice::close()
{
    std::lock_guard<std::mutex> guard(m_deviceMutex);
    closeCommunications();
    return true;
}

bool FloatingBaseEstimatorDevice::loadTransformBroadcaster()
{
    constexpr auto logPrefix = "[FloatingBaseEstimatorDevice::loadTransformBroadcaster]";
    yarp::os::Property tfBroadcasterSettings{{"device", yarp::os::Value("transformClient")},
                                             {"remote", yarp::os::Value("/transformServer")},
                                             {"local", yarp::os::Value(m_portPrefix + "/transformClient")}};

    if (!m_transformBroadcaster.open(tfBroadcasterSettings))
    {
        log()->error("{} could not open transform broadcaster.", logPrefix);
        return false;
    }

    if (!m_transformBroadcaster.view(m_transformInterface))
    {
        log()->error("{} could not access transform interface", logPrefix);
        return false;
    }

    return true;
}
