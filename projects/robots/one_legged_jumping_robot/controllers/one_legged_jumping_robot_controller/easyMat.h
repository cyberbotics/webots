// Copyright 1996-2019 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/***************************************************************************

  Description : A simple matrix operation library
  Authors     : Yu Xianyuan
  Email       : xy_yu_beijing@163.com
  
***************************************************************************/
#ifndef _EASYMAT_H_
#define _EASYMAT_H_

#include <stdint.h>

//-----------------------------------------------------------typedef
/*
矩阵 类型
*/
typedef struct
{
  uint16_t row;
  uint16_t col;
  double** data;
}matTypeDef;
//-----------------------------------------------------------function
extern void easyMat_create    (matTypeDef* mat, uint16_t row, uint16_t col)              ;
extern void easyMat_free      (matTypeDef* mat)                                          ;
extern void easyMat_init      (matTypeDef* mat, double* data)                            ;
extern void easyMat_clear     (matTypeDef* mat)                                          ;
extern void easyMat_eye       (matTypeDef* mat)                                          ;
extern void easyMat_add       (matTypeDef* outMat, matTypeDef* mat1, matTypeDef* mat2)   ;
extern void easyMat_sub       (matTypeDef* outMat, matTypeDef* mat1, matTypeDef* mat2)   ;
extern void easyMat_mult      (matTypeDef* outMat, matTypeDef* mat1, matTypeDef* mat2)   ;
extern void easyMat_trans     (matTypeDef* dstMat, matTypeDef* srcMat)                   ;
extern void easyMat_copy      (matTypeDef* dstMat, matTypeDef* srcMat)                   ;
//extern void easyMat_inv       (matTypeDef* dstMat, matTypeDef* srcMat)                  ;
extern void easyMat_rotX      (matTypeDef* Mat, double angle)                            ;
extern void easyMat_rotY      (matTypeDef* Mat, double angle)                            ;
extern void easyMat_rotZ      (matTypeDef* Mat, double angle)                            ;
extern void easyMat_RPY       (matTypeDef* outRot, double roll, double pitch, double yaw);



#endif

