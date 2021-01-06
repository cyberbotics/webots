/*
 * Copyright 1996-2021 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  The defines common to both pipe.c and client.c.
 */

#ifndef COMMON_H
#define COMMON_H

/* The Khepera robot has 8 infra red sensors */
#define NB_IR_SENSOR 8

/* we use a 64 millisecond time step */
#define TIME_STEP 64

#define PIPE_FILE_DIRECTORY "/tmp/"

#endif /* COMMON_H */
