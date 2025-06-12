//
// Created by user on 2025/4/22.
//

#include "atomic.h"

void atomic_bool_set(AtomicBool* atomic, bool value) {
    portENTER_CRITICAL(&atomic->mutex);
    atomic->value = value;
    portEXIT_CRITICAL(&atomic->mutex);
}

bool atomic_bool_get(AtomicBool* atomic) {
    portENTER_CRITICAL(&atomic->mutex);
    const bool value = atomic->value;
    portEXIT_CRITICAL(&atomic->mutex);
    return value;
}


void atomic_int_set(AtomicInt* atomic, int value) {
    portENTER_CRITICAL(&atomic->mutex);
    atomic->value = value;
    portEXIT_CRITICAL(&atomic->mutex);
}

int atomic_int_get(AtomicInt* atomic) {
    portENTER_CRITICAL(&atomic->mutex);
    const int value = atomic->value;
    portEXIT_CRITICAL(&atomic->mutex);
    return value;
}


void atomic_ptr_set(AtomicPointer* atomic, void* ptr) {
    portENTER_CRITICAL(&atomic->mutex);
    atomic->ptr = ptr;
    portEXIT_CRITICAL(&atomic->mutex);
}

void* atomic_ptr_get(AtomicPointer* atomic) {
    portENTER_CRITICAL(&atomic->mutex);
    void* ptr = atomic->ptr;
    portEXIT_CRITICAL(&atomic->mutex);
    return ptr;
}
