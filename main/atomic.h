//
// Created by user on 2025/4/22.
//

#ifndef ATOMIC_H
#define ATOMIC_H

#include <portmacro.h>

typedef struct {
    bool value;
    portMUX_TYPE mutex;
} AtomicBool;

#define atomic_bool_init(value) {value, portMUX_INITIALIZER_UNLOCKED}
void atomic_bool_set(AtomicBool* atomic, bool value);
bool atomic_bool_get(AtomicBool* atomic);

typedef struct {
    int value;
    portMUX_TYPE mutex;
} AtomicInt;
#define atomic_int_init(value) {value, portMUX_INITIALIZER_UNLOCKED}
void atomic_int_set(AtomicInt* atomic, int value);
int atomic_int_get(AtomicInt* atomic);

typedef struct {
    void* ptr;
    portMUX_TYPE mutex;
} AtomicPointer;

#define atomic_ptr_init(ptr) {ptr, portMUX_INITIALIZER_UNLOCKED}
void atomic_ptr_set(AtomicPointer* atomic, void* ptr);
void* atomic_ptr_get(AtomicPointer* atomic);

#endif //ATOMIC_H
