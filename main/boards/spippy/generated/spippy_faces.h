#pragma once

#include <stddef.h>
#include <stdint.h>

#include <lvgl.h>

typedef struct {
    const char *name;
    const lv_image_dsc_t *image;
} spippy_face_resource_t;

const lv_image_dsc_t *spippy_face_find(const char *name);
bool spippy_face_exists(const char *name);
const spippy_face_resource_t *spippy_face_resources(size_t *count);
