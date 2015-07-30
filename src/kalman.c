#include "kalman.h"
#include "arm_math.h"

static arm_matrix_instance_f32 A;
static arm_matrix_instance_f32 B;
static arm_matrix_instance_f32 C;

static arm_matrix_instance_f32 V;
static arm_matrix_instance_f32 W;

static arm_matrix_instance_f32 P_pri;
static arm_matrix_instance_f32 P_post;

static arm_matrix_instance_f32 X_pri;

static arm_matrix_instance_f32 K;


void updateModel();
