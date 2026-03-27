#include <Python.h>
#include "zp_params.hpp"
#include "system_manager.hpp"
#include "telemetry_manager.hpp"
#include "attitude_manager.hpp"
#include "sitl_drivers/sitl_systemutils.hpp"
#include "sitl_drivers/sitl_iwdg.hpp"
#include "sitl_drivers/sitl_logger.hpp"
#include "sitl_drivers/sitl_rc.hpp"
#include "sitl_drivers/sitl_powermodule.hpp"
#include "sitl_drivers/sitl_telemlink.hpp"
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

std::queue<std::string> telemTxMessages;
std::queue<std::string> telemRxMessages;
std::mutex telemMutex;

static void telemLogCallback(const std::string& message, uint8_t direction) {
    std::lock_guard<std::mutex> lock(telemMutex);
if (direction == 1) {
    telemTxMessages.push(message);
    if (telemTxMessages.size() > 100) {
        telemTxMessages.pop(); // Limit the queue size to 100 messages
}
    } else if (direction == 0) {
        telemRxMessages.push(message);
        if (telemRxMessages.size() > 100) {
            telemRxMessages.pop();
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
    SITL_TELEM* telem;
    SITL_IMU* imu;
    SITL_GPS* gps;
    SITL_Motor* leftRollMotor;
    SITL_Motor* pitchMotor;
    SITL_Motor* throttleMotor;
    SITL_Motor* yawMotor;
    SITL_Motor* rightRollMotor;
    SITL_Motor* leftFlapMotor;
    SITL_Motor* rightFlapMotor;
    SITL_Motor* steerMotor;
    
    MotorInstance_t motors[8];
    MotorGroupInstance_t motorGroup;
    
    uint32_t sitlRateHz;
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
    delete self->telem;
    delete self->imu;
    delete self->gps;
    delete self->leftRollMotor;
    delete self->pitchMotor;
    delete self->throttleMotor;
    delete self->yawMotor;
    delete self->rightRollMotor;
    delete self->leftFlapMotor;
    delete self->rightFlapMotor;
    delete self->steerMotor;
    Py_TYPE(self)->tp_free((PyObject*)self);
}

static PyObject* ZP_new(PyTypeObject* type, PyObject* args, PyObject* kwds) {
    const char* ip = nullptr;
    int port = 0;
    uint32_t sitlRateHz = 1000;
   
    // Parse arguments from Python
    static char* kwlist[] = {(char*)"sitl_rate_hz", (char*)"ip", (char*)"port", NULL};
    if (!PyArg_ParseTupleAndKeywords(args, kwds, "|isi", kwlist, &sitlRateHz, &ip, &port)) {
        return NULL;
    }
    
    ZPObject* self = (ZPObject*)type->tp_alloc(type, 0);
    if (self != NULL) {
        ZP_PARAM::init();

        self->sysUtils = new SITL_SystemUtils();
        self->amQueue = new SITL_Queue<RCMotorControlMessage_t>();
        self->tmQueue = new SITL_Queue<TMMessage_t>();
        self->logQueue = new SITL_LogQueue();
        self->mavlinkQueue = new SITL_Queue<mavlink_message_t>();
        
        self->iwdg = new SITL_IWDG();
        self->logger = new SITL_Logger();
        self->rc = new SITL_RC();
        self->pm = new SITL_PowerModule();
        self->telem = new SITL_TELEM(ip, port, telemLogCallback);
        self->imu = new SITL_IMU();
        self->gps = new SITL_GPS();
        self->leftRollMotor = new SITL_Motor(1);
        self->pitchMotor = new SITL_Motor(2);
        self->throttleMotor = new SITL_Motor(3);
        self->yawMotor = new SITL_Motor(4);
        self->rightRollMotor = new SITL_Motor(5);
        self->leftFlapMotor = new SITL_Motor(6);
        self->rightFlapMotor = new SITL_Motor(7);
        self->steerMotor = new SITL_Motor(8);
        
        self->motors[0] = {self->leftRollMotor, false, 50, 0, 100, MotorFunction_e::AILERON};
        self->motors[1] = {self->pitchMotor, false, 50, 0, 100, MotorFunction_e::ELEVATOR};
        self->motors[2] = {self->throttleMotor, false, 50, 0, 100, MotorFunction_e::THROTTLE};
        self->motors[3] = {self->yawMotor, false, 50, 0, 100, MotorFunction_e::RUDDER};
        self->motors[4] = {self->rightRollMotor, true, 50, 0, 100, MotorFunction_e::AILERON};
        self->motors[5] = {self->leftFlapMotor, false, 50, 0, 100, MotorFunction_e::FLAP};
        self->motors[6] = {self->rightFlapMotor, false, 50, 0, 100, MotorFunction_e::FLAP};
        self->motors[7] = {self->steerMotor, false, 50, 0, 100, MotorFunction_e::GROUND_STEERING};

        self->motorGroup = {self->motors, 8};
        
        self->imu->init();
        
        self->sm = new SystemManager(
            self->sysUtils, self->iwdg, self->logger, self->rc, self->pm,
            self->amQueue, self->tmQueue, self->logQueue
        );
        
        self->tm = new TelemetryManager(
            self->sysUtils, self->telem, self->tmQueue, self->amQueue, self->mavlinkQueue
        );
        
        self->am = new AttitudeManager(
            self->sysUtils, self->gps, self->imu,
            self->amQueue, self->tmQueue, self->logQueue,
            &self->motorGroup
        );
        
        self->sitlRateHz = sitlRateHz;
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
    float roll, pitch, yaw, throttle, arm, flap, fltmode;
    if (!PyArg_ParseTuple(args, "fffffff", &roll, &pitch, &yaw, &throttle, &arm, &flap, &fltmode))
        return NULL;
    
    self->rc->update_from_commands(roll, pitch, yaw, throttle, arm, flap, fltmode);
    Py_RETURN_NONE;
}

static PyObject* ZP_update(ZPObject* self, PyObject* args) {
    if (self->smCounter % (self->sitlRateHz / SM_SCHEDULING_RATE_HZ) == 0) {
        self->sm->smUpdate();
    }
    
    if (self->tmCounter % (self->sitlRateHz / TM_SCHEDULING_RATE_HZ) == 0) {
        self->tm->tmUpdate();
    }
    
    if (self->amCounter % (self->sitlRateHz / AM_SCHEDULING_RATE_HZ) == 0) {
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
    uint32_t roll = self->leftRollMotor->get();
    uint32_t pitch = self->pitchMotor->get();
    uint32_t yaw = self->yawMotor->get();
    uint32_t throttle = self->throttleMotor->get();
    uint32_t flap = self->leftFlapMotor->get();
    uint32_t steer = self->steerMotor->get();
    
    return Py_BuildValue("(iiiiii)", roll, pitch, yaw, throttle, flap, steer);
}

static PyObject* ZP_getTelemMessages(ZPObject* self, PyObject* args) {
    std::lock_guard<std::mutex> lock(telemMutex);
    PyObject* list = PyList_New(0);

    while (!telemTxMessages.empty()) {
        PyObject* tuple = Py_BuildValue("(is)", 1, telemTxMessages.front().c_str());
        PyList_Append(list, tuple);
        Py_DECREF(tuple);
        telemTxMessages.pop();
    }
    
    while (!telemRxMessages.empty()) {
        PyObject* tuple = Py_BuildValue("(is)", 0, telemRxMessages.front().c_str());
        PyList_Append(list, tuple);
        Py_DECREF(tuple);
        telemRxMessages.pop();
    }

    return list;
}

static PyMethodDef ZP_methods[] = {
    {"update_from_plant", (PyCFunction)ZP_updateFromPlant, METH_VARARGS, "Update sensors from plant"},
    {"set_max_batt_capacity", (PyCFunction)ZP_setBatteryCapacity, METH_VARARGS, "Set max battery capacity"},
    {"set_rc", (PyCFunction)ZP_setRC, METH_VARARGS, "Set RC commands"},
    {"update", (PyCFunction)ZP_update, METH_NOARGS, "Run all managers"},
    {"get_motor_outputs", (PyCFunction)ZP_getMotorOutputs, METH_NOARGS, "Get motor outputs"},
    {"get_telem_messages", (PyCFunction)ZP_getTelemMessages, METH_NOARGS, "Get TELEM messages"},
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
