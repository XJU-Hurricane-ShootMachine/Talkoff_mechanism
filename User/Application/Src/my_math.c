/**
 * @file    my_math.c
 * @author  Deadline039
 * @brief   精简数学库, 封装一些常用的函数
 * @version 0.1
 * @date    2024-03-02
 */

#include "my_math.h"
#include "float.h"

/**
 * @brief 比较两个`float`类型的浮点数
 *
 * @param x 第一个浮点数
 * @param y 第二个浮点数
 * @return fp_compare_result_t
 */
fp_compare_result_t math_compare_float(float x, float y) {
    if ((x - y) > FLT_EPSILON) {
        return MATH_FP_MORETHAN;
    } else if ((x - y) < FLT_EPSILON) {
        return MATH_FP_LESSTHAN;
    } else {
        return MATH_FP_EQUATION;
    }
}

/**
 * @brief 比较两个`double`类型的浮点数
 *
 * @param x 第一个浮点数
 * @param y 第二个浮点数
 * @return fp_compare_result_t
 */
fp_compare_result_t math_compare_double(double x, double y) {
    if ((x - y) > DBL_EPSILON) {
        return MATH_FP_MORETHAN;
    } else if ((x - y) < DBL_EPSILON) {
        return MATH_FP_LESSTHAN;
    } else {
        return MATH_FP_EQUATION;
    }
}
