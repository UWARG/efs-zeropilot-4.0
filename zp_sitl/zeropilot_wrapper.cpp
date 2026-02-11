#include <Python.h>
#include "system_manager.hpp"
#include "telemetry_manager.hpp"
#include "attitude_manager.hpp"
#include "sitl_drivers/sitl_systemutils.hpp"
#include "sitl_drivers/sitl_iwdg.hpp"
#include "sitl_drivers/sitl_logger.hpp"
#include "sitl_drivers/sitl_rc.hpp"
#include "sitl_drivers/sitl_powermodule.hpp"
#include "sitl_drivers/sitl_rfd.hpp"
#include "sitl_drivers/sitl_imu.hpp"
#include "sitl_drivers/sitl_gps.hpp"
#include "sitl_drivers/sitl_queue.hpp"
#include "sitl_drivers/sitl_logqueue.hpp"
#include "sitl_drivers/sitl_motor.hpp"
#include <functional>
#include <string>
#include <queue>
#include <mutex>

#define SM_SCHEDULING_RATE_HZ 20
#define TM_SCHEDULING_RATE_HZ 20
#define AM_SCHEDULING_RATE_HZ 100

std::queue<std::string> rfdTxMessages;
std::queue<std::string> rfdRxMessages;
std::mutex rfdMutex;

static void telemLogCallback(const std::string& message, uint8_t direction) {
    std::lock_guard<std::mutex> lock(rfdMutex);
if (direction == 1) {
    rfdTxMessages.push(message);
    if (rfdTxMessages.size() > 100) {
        rfdTxMessages.pop(); // Limit the queue size to 100 messages
}
    } else if (direction == 0) {
        rfdRxMessages.push(message);
        if (rfdRxMessages.size() > 100) {
            rfdRxMessages.pop();
        }
    }
}

typedef struct {
    PyObject_HEAD
    
    SystemManager* sm;
    TelemetryManager* tm;
    AttitudeManager* am;
    
    SITL_SystemUtils* sysUtils;
    SITL_Queue<RCMotorControlMessage_t>* amQueue;
    SITL_Queue<TMMessage_t>* tmQueue;
    SITL_LogQueue* logQueue;
    SITL_Queue<mavlink_message_t>* mavlinkQueue;
    
    SITL_IWDG* iwdg;
    SITL_Logger* logger;
    SITL_RC* rc;
    SITL_PowerModule* pm;
    SITL_RFD* rfd;
    SITL_IMU* imu;
    SITL_GPS* gps;
    SITL_Motor* rollMotor;
    SITL_Motor* pitchMotor;
    SITL_Motor* yawMotor;
    SITL_Motor* throttleMotor;
    
    MotorInstance_t rollMotorInstance;
    MotorInstance_t pitchMotorInstance;
    MotorInstance_t yawMotorInstance;
    MotorInstance_t throttleMotorInstance;
    MotorGroupInstance_t rollGroup;
    MotorGroupInstance_t pitchGroup;
    MotorGroupInstance_t yawGroup;
    MotorGroupInstance_t throttleGroup;
    MotorGroupInstance_t flapGroup;
    MotorGroupInstance_t steeringGroup;
    
    uint32_t smCounter;
    uint32_t tmCounter;
    uint32_t amCounter;
} ZPObject;

static void ZP_dealloc(ZPObject* self) {
    delete self->sm;
    delete self->tm;
    delete self->am;
    delete self->sysUtils;
    delete self->amQueue;
    delete self->tmQueue;
    delete self->logQueue;
    delete self->mavlinkQueue;
    delete self->iwdg;
    delete self->logger;
    delete self->rc;
    delete self->pm;
    delete self->rfd;
    delete self->imu;
    delete self->gps;
    delete self->rollMotor;
    delete self->pitchMotor;
    delete self->yawMotor;
    delete self->throttleMotor;
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject* ZP_new(PyTypeObject* type, PyObject* args, PyObject* kwds) {
    const char* ip = nullptr;
    int port = 0;
   
    // Parse arguments from Python
    static char* kwlist[] = {(char*)"ip", (char*)"port", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|si", kwlist, &ip, &port)) {
        return NULL;
    }
    
    ZPObject* self = (ZPObject*)type->tp_alloc(type, 0);
    if (self != NULL) {
        self->sysUtils = new SITL_SystemUtils();
        self->amQueue = new SITL_Queue<RCMotorControlMessage_t>();
        self->tmQueue = new SITL_Queue<TMMessage_t>();
        self->logQueue = new SITL_LogQueue();
        self->mavlinkQueue = new SITL_Queue<mavlink_message_t>();
        
        self->iwdg = new SITL_IWDG();
        self->logger = new SITL_Logger();
        self->rc = new SITL_RC();
        self->pm = new SITL_PowerModule();
        self->rfd = new SITL_RFD(ip, port, telemLogCallback);
        self->imu = new SITL_IMU();
        self->gps = new SITL_GPS();
        self->rollMotor = new SITL_Motor();
        self->pitchMotor = new SITL_Motor();
        self->yawMotor = new SITL_Motor();
        self->throttleMotor = new SITL_Motor();
        
        self->rollMotorInstance = {self->rollMotor, false};
        self->pitchMotorInstance = {self->pitchMotor, false};
        self->yawMotorInstance = {self->yawMotor, false};
        self->throttleMotorInstance = {self->throttleMotor, false};
        
        self->rollGroup = {&self->rollMotorInstance, 1};
        self->pitchGroup = {&self->pitchMotorInstance, 1};
        self->yawGroup = {&self->yawMotorInstance, 1};
        self->throttleGroup = {&self->throttleMotorInstance, 1};
        self->flapGroup = {nullptr, 0};
        self->steeringGroup = {nullptr, 0};
        
        self->imu->init();
        
        self->sm = new SystemManager(
            self->sysUtils, self->iwdg, self->logger, self->rc, self->pm,
            self->amQueue, self->tmQueue, self->logQueue
        );
        
        self->tm = new TelemetryManager(
            self->sysUtils, self->rfd, self->tmQueue, self->amQueue, self->mavlinkQueue
        );
        
        self->am = new AttitudeManager(
            self->sysUtils, self->gps, self->imu,
            self->amQueue, self->tmQueue, self->logQueue,
            &self->rollGroup, &self->pitchGroup, &self->yawGroup,
            &self->throttleGroup, &self->flapGroup, &self->steeringGroup
        );
        
        self->smCounter = 0;
        self->tmCounter = 0;
        self->amCounter = 0;
    }
    return (PyObject*)self;
}

static PyObject* ZP_updateFromPlant(ZPObject* self, PyObject* args) {
    double roll_rad, pitch_rad;
    double p_rad_s, q_rad_s, r_rad_s;
    double lat_deg, lon_deg, alt_m, ground_speed_mps, course_deg;
    float fuel_lbs, rpm;
    
    if (!PyArg_ParseTuple(args, "ddddddddddff",
        &roll_rad, &pitch_rad,
        &p_rad_s, &q_rad_s, &r_rad_s,
        &lat_deg, &lon_deg, &alt_m, &ground_speed_mps, &course_deg,
        &fuel_lbs, &rpm))
        return NULL;
    
    self->imu->update_from_plant(roll_rad, pitch_rad, p_rad_s, q_rad_s, r_rad_s);
    self->gps->update_from_plant(lat_deg, lon_deg, alt_m, ground_speed_mps, course_deg);
    self->pm->update_from_plant(fuel_lbs, rpm);
    
    Py_RETURN_NONE;
}

static PyObject* ZP_setBatteryCapacity(ZPObject* self, PyObject* args) {
    float capacity;
    if (!PyArg_ParseTuple(args, "f", &capacity))
        return NULL;
    self->pm->set_max_batt_capacity(capacity);
    Py_RETURN_NONE;
}

static PyObject* ZP_setRC(ZPObject* self, PyObject* args) {
    float roll, pitch, yaw, throttle, arm;
    if (!PyArg_ParseTuple(args, "fffff", &roll, &pitch, &yaw, &throttle, &arm))
        return NULL;
    
    self->rc->update_from_commands(roll, pitch, yaw, throttle, arm);
    Py_RETURN_NONE;
}

static PyObject* ZP_update(ZPObject* self, PyObject* args) {
    if (self->smCounter % (1000/SM_SCHEDULING_RATE_HZ) == 0) {
        self->sm->smUpdate();
    }
    
    if (self->tmCounter % (1000/TM_SCHEDULING_RATE_HZ) == 0) {
        self->tm->tmUpdate();
    }
    
    if (self->amCounter % (1000/AM_SCHEDULING_RATE_HZ) == 0) {
        self->am->amUpdate();
    }
    
    self->smCounter++;
    self->tmCounter++;
    self->amCounter++;

    if (self->iwdg->check_watchdog() == false) {
        Py_RETURN_FALSE; // Report false if watchdog times out
    }
    
    Py_RETURN_TRUE;
}

static PyObject* ZP_getMotorOutputs(ZPObject* self, PyObject* args) {
    uint32_t roll = self->rollMotor->get();
    uint32_t pitch = self->pitchMotor->get();
    uint32_t yaw = self->yawMotor->get();
    uint32_t throttle = self->throttleMotor->get();
    
    return Py_BuildValue("(iiii)", roll, pitch, yaw, throttle);
}

static PyObject* ZP_getRFDMessages(ZPObject* self, PyObject* args) {
    std::lock_guard<std::mutex> lock(rfdMutex);
    PyObject* list = PyList_New(0);

    while (!rfdTxMessages.empty()) {
        PyObject* tuple = Py_BuildValue("(is)", 1, rfdTxMessages.front().c_str());
        PyList_Append(list, tuple);
        Py_DECREF(tuple);
        rfdTxMessages.pop();
    }
    
    while (!rfdRxMessages.empty()) {
        PyObject* tuple = Py_BuildValue("(is)", 0, rfdRxMessages.front().c_str());
        PyList_Append(list, tuple);
        Py_DECREF(tuple);
        rfdRxMessages.pop();
    }

    return list;
}

static PyMethodDef ZP_methods[] = {
    {"update_from_plant", (PyCFunction)ZP_updateFromPlant, METH_VARARGS, "Update sensors from plant"},
    {"set_max_batt_capacity", (PyCFunction)ZP_setBatteryCapacity, METH_VARARGS, "Set max battery capacity"},
    {"set_rc", (PyCFunction)ZP_setRC, METH_VARARGS, "Set RC commands"},
    {"update", (PyCFunction)ZP_update, METH_NOARGS, "Run all managers"},
    {"get_motor_outputs", (PyCFunction)ZP_getMotorOutputs, METH_NOARGS, "Get motor outputs"},
    {"get_rfd_messages", (PyCFunction)ZP_getRFDMessages, METH_NOARGS, "Get RFD messages"},
    {NULL}
};

static PyTypeObject ZPType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "zeropilot.ZeroPilot",
    sizeof(ZPObject),
    0,
    (destructor)ZP_dealloc,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    Py_TPFLAGS_DEFAULT,
    0, 0, 0, 0, 0, 0, 0,
    ZP_methods,
    0, 0, 0, 0, 0, 0, 0, 0, 0,
    ZP_new,
};

static PyModuleDef zp_module = {
    PyModuleDef_HEAD_INIT,
    "zeropilot",
    0,
    -1,
    0, 0, 0, 0, 0
};

PyMODINIT_FUNC PyInit_zeropilot(void) {
    PyObject* m;
    if (PyType_Ready(&ZPType) < 0)
        return NULL;
    
    m = PyModule_Create(&zp_module);
    if (m == NULL)
        return NULL;
    
    Py_INCREF(&ZPType);
    PyModule_AddObject(m, "ZeroPilot", (PyObject*)&ZPType);
    return m;
}
