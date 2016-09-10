//
//  sm.h
//  TwinRotors
//
//  Created on 9/6/16.
//  Contributors:
//      Guilherme Felipe da Silva
//      André Luiz Luppi
//      Felipe Hartcopp Betoni
//

#ifndef SM_H
#define SM_H

//Function pointer
typedef void (*function_pointer)(void *);

typedef struct{
    function_pointer ptr;
    unsigned char first_time;
}state_machine;

//Definitions
#define STATE(name)         void name(state_machine *_sm_)
#define NEXT_STATE(name)    _sm_->ptr = (function_pointer)name
#define INIT(sm,name)       {sm.ptr = (function_pointer)name;sm.first_time=1;}
#define EXEC(sm)            {function_pointer temp=sm.ptr;sm.ptr(&sm);sm.first_time=(temp != sm.ptr);}
#define FIRST               (_sm_->first_time)
#define COMPARE(sm,name)	(sm.ptr == (function_pointer)name)

#endif // SM_H
