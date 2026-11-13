set(srcs
        "main.c"
       
       
)

set(requires "bluepad32" "bluepad32_arduino" "arduino" "btstack" "callbacks")

idf_component_register(SRCS "arduino_main.cpp" "autonomous.cpp" "servo.cpp" "motors.cpp" "lineSensor.cpp" "colorSensor.cpp" "${srcs}"
                    INCLUDE_DIRS "." "../include"
                    REQUIRES "${requires}")
