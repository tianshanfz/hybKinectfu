/*
 * cpu_declar.h
 *
 *  Created on: May 24, 2016
 *      Author: hanyinbo
 */

#ifndef CPU_DECLAR_H_
#define CPU_DECLAR_H_




#ifndef SAFE_DELETE
#define SAFE_DELETE(p) {if(p != nullptr) { delete (p); (p) = nullptr; } } //Delete object by New create
#endif

#ifndef SAFE_DELETE_ARR     //Delete Arrary
#define SAFE_DELETE_ARR(p) {if(p != nullptr) {delete[] (p); (p) = nullptr;}}
#endif

/*#ifndef INDEXN(x,y,n)
#define INDEXN(x,y,n) ((x)+(y)*(n))
#endif
#ifndef M_PI
#define M_PI 3.14159265
#endif*/

#endif /* CPU_DECLAR_H_ */
