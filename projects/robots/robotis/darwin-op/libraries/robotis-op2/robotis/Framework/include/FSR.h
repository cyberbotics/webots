/*
 * FSR.h
 *
 *  Created on: 2011. 5. 12.
 *      Author: ROBOTIS
 */

#ifndef FSR_H_
#define FSR_H_

namespace Robot
{
    class FSR
    {
    public:
        static const int ID_R_FSR = 111;
        static const int ID_L_FSR = 112;

        enum
        {
            P_MODEL_NUMBER_L            = 0,
            P_MODEL_NUMBER_H            = 1,
            P_VERSION                   = 2,
            P_ID                        = 3,
            P_BAUD_RATE                 = 4,
            P_RETURN_DELAY_TIME         = 5,
            P_RETURN_LEVEL              = 16,
            P_OPERATING_MODE            = 19,
            P_LED                       = 25,
            P_FSR1_L                    = 26,
            P_FSR1_H                    = 27,
            P_FSR2_L                    = 28,
            P_FSR2_H                    = 29,
            P_FSR3_L                    = 30,
            P_FSR3_H                    = 31,
            P_FSR4_L                    = 32,
            P_FSR4_H                    = 33,
            P_FSR_X                     = 34,
            P_FSR_Y                     = 35,
            P_PRESENT_VOLTAGE           = 42,
            P_REGISTERED_INSTRUCTION    = 44,
            P_LOCK                      = 47,
            MAXNUM_ADDRESS
        };
    };
}

#endif /* FSR_H_ */
