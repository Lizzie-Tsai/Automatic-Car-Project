class COMMAND:
    CMD_MOTOR      =  "CMD_MOTOR"
    CMD_LED        =  "CMD_LED"
    CMD_LED_MOD    =  "CMD_LED_MOD"
    CMD_SERVO      =  "CMD_SERVO"
    CMD_BUZZER     =  "CMD_BUZZER"
    CMD_POWER      =  "CMD_POWER"

    CMD_VIDEO      =  "CMD_VIDEO"
    CMD_MATRIX_MOD =  "CMD_MATRIX_MOD"

    CMD_LIGHT      =  "CMD_LIGHT"
    CMD_TRACK      =  "CMD_TRACK"
    CMD_OVERRIDE   =  "CMD_OVERRIDE"
    def __init__(self):
        pass

class STATE:
    STATE_O      =  "STATE_O"
    STATE_T      =  "STATE_T"
    STATE_OS     =  "STATE_OS"
    STATE_TS     =  "STATE_TS"
    STATE_BS     =  "STATE_BS"
    STATE_RS     =  "STATE_RS"
    STATE_NONE   =  "STATE_NONE"

    def __init__(self):
        pass

class STATE_STRING:
    IN_MOTION_O      =  "IN_MOTION : AUTO"
    IN_MOTION_T      =  "IN_MOTION : TRACK"
    MOTIONLESS       =  "MOTIONLESS"
    OVERRIDE_1       =  "OVERRIDE 1"
    OVERRIDE_2       =  "OVERRIDE 2"
    NONE             =  "None"

    def __init__(self):
        pass

class CAUSE_STRING:
    OS     =  "OBSTACLE DETECTED"
    TS     =  "NO TRACK"
    BS     =  "BLUETOOTH OVERRIDE"
    RS     =  "CONTROL PANEL OVERRIDE"
    NONE   =  "None"

    def __init__(self):
        pass

