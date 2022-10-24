/*
 * Copyright (c) 2020 Stephanos Ioannidis <root@stephanos.io>
 * Copyright (C) 2010-2020 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <arm_math.h>
#include "../../common/test_common.h"

#include "q31.pat"

#define SNR_ERROR_THRESH	((float32_t)100)
#define ABS_ERROR_THRESH_Q31	((q31_t)100)
#define ABS_ERROR_THRESH_Q63	((q63_t)(1 << 18))

ZTEST_SUITE(complexmath_q31, NULL, NULL, NULL, NULL, NULL);

static void test_arm_cmplx_conj_q31(
	const q31_t *input1, const q31_t *ref, size_t length)
{
	size_t buf_length;
	q31_t *output;

	/* Complex number buffer length is twice the data length */
	buf_length = 2 * length;

	/* Allocate output buffer */
	output = malloc(buf_length * sizeof(q31_t));
	zassert_not_null(output, ASSERT_MSG_BUFFER_ALLOC_FAILED);

	/* Run test function */
	arm_cmplx_conj_q31(input1, output, length);

	/* Validate output */
	zassert_true(
		test_snr_error_q31(buf_length, output, ref, SNR_ERROR_THRESH),
		ASSERT_MSG_SNR_LIMIT_EXCEED);

	zassert_true(
		test_near_equal_q31(buf_length, output, ref,
			ABS_ERROR_THRESH_Q31),
		ASSERT_MSG_ABS_ERROR_LIMIT_EXCEED);

	/* Free output buffer */
	free(output);
}

DEFINE_TEST_VARIANT3(complexmath_q31, arm_cmplx_conj_q31, 3, in_com1, ref_conj, 3);
DEFINE_TEST_VARIANT3(complexmath_q31, arm_cmplx_conj_q31, 8, in_com1, ref_conj, 8);
DEFINE_TEST_VARIANT3(complexmath_q31, arm_cmplx_conj_q31, 11, in_com1, ref_conj, 11);

static void test_arm_cmplx_dot_prod_q31(
	const q31_t *input1, const q31_t *input2, const q63_t *ref,
	size_t length)
{
	q63_t *output;

	/* Allocate output buffer */
	output = malloc(2 * sizeof(q63_t));
	zassert_not_null(output, ASSERT_MSG_BUFFER_ALLOC_FAILED);

	/* Run test function */
	arm_cmplx_dot_prod_q31(input1, input2, length, &output[0], &output[1]);

	/* Validate output */
	zassert_true(
		test_snr_error_q63(2, output, ref, SNR_ERROR_THRESH),
		ASSERT_MSG_SNR_LIMIT_EXCEED);

	zassert_true(
		test_near_equal_q63(2, output, ref, ABS_ERROR_THRESH_Q63),
		ASSERT_MSG_ABS_ERROR_LIMIT_EXCEED);

	/* Free output buffer */
	free(output);
}

DEFINE_TEST_VARIANT4(complexmath_q31, arm_cmplx_dot_prod_q31, 3, in_com1, in_com2, ref_dot_prod_3,
		     3);
DEFINE_TEST_VARIANT4(complexmath_q31, arm_cmplx_dot_prod_q31, 8, in_com1, in_com2, ref_dot_prod_4n,
		     8);
DEFINE_TEST_VARIANT4(complexmath_q31, arm_cmplx_dot_prod_q31, 11, in_com1, in_com2,
		     ref_dot_prod_4n1, 11);

static void test_arm_cmplx_mag_q31(
	const q31_t *input1, const q31_t *ref, size_t length)
{
	q31_t *output;

	/* Allocate output buffer */
	output = malloc(length * sizeof(q31_t));
	zassert_not_null(output, ASSERT_MSG_BUFFER_ALLOC_FAILED);

	/* Run test function */
	arm_cmplx_mag_q31(input1, output, length);

	/* Validate output */
	zassert_true(
		test_snr_error_q31(length, output, ref, SNR_ERROR_THRESH),
		ASSERT_MSG_SNR_LIMIT_EXCEED);

	zassert_true(
		test_near_equal_q31(length, output, ref, ABS_ERROR_THRESH_Q31),
		ASSERT_MSG_ABS_ERROR_LIMIT_EXCEED);

	/* Free output buffer */
	free(output);
}

DEFINE_TEST_VARIANT3(complexmath_q31, arm_cmplx_mag_q31, 3, in_com1, ref_mag, 3);
DEFINE_TEST_VARIANT3(complexmath_q31, arm_cmplx_mag_q31, 8, in_com1, ref_mag, 8);
DEFINE_TEST_VARIANT3(complexmath_q31, arm_cmplx_mag_q31, 11, in_com1, ref_mag, 11);

static void test_arm_cmplx_mag_squared_q31(
	const q31_t *input1, const q31_t *ref, size_t length)
{
	q31_t *output;

	/* Allocate output buffer */
	output = malloc(length * sizeof(q31_t));
	zassert_not_null(output, ASSERT_MSG_BUFFER_ALLOC_FAILED);

	/* Run test function */
	arm_cmplx_mag_squared_q31(input1, output, length);

	/* Validate output */
	zassert_true(
		test_snr_error_q31(length, output, ref, SNR_ERROR_THRESH),
		ASSERT_MSG_SNR_LIMIT_EXCEED);

	zassert_true(
		test_near_equal_q31(length, output, ref, ABS_ERROR_THRESH_Q31),
		ASSERT_MSG_ABS_ERROR_LIMIT_EXCEED);

	/* Free output buffer */
	free(output);
}

DEFINE_TEST_VARIANT3(complexmath_q31, arm_cmplx_mag_squared_q31, 3, in_com1, ref_mag_squared, 3);
DEFINE_TEST_VARIANT3(complexmath_q31, arm_cmplx_mag_squared_q31, 8, in_com1, ref_mag_squared, 8);
DEFINE_TEST_VARIANT3(complexmath_q31, arm_cmplx_mag_squared_q31, 11, in_com1, ref_mag_squared, 11);

static void test_arm_cmplx_mult_cmplx_q31(
	const q31_t *input1, const q31_t *input2, const q31_t *ref,
	size_t length)
{
	size_t buf_length;
	q31_t *output;

	/* Complex number buffer length is twice the data length */
	buf_length = 2 * length;

	/* Allocate output buffer */
	output = malloc(buf_length * sizeof(q31_t));
	zassert_not_null(output, ASSERT_MSG_BUFFER_ALLOC_FAILED);

	/* Run test function */
	arm_cmplx_mult_cmplx_q31(input1, input2, output, length);

	/* Validate output */
	zassert_true(
		test_snr_error_q31(buf_length, output, ref, SNR_ERROR_THRESH),
		ASSERT_MSG_SNR_LIMIT_EXCEED);

	zassert_true(
		test_near_equal_q31(buf_length, output, ref,
			ABS_ERROR_THRESH_Q31),
		ASSERT_MSG_ABS_ERROR_LIMIT_EXCEED);

	/* Free output buffer */
	free(output);
}

DEFINE_TEST_VARIANT4(complexmath_q31, arm_cmplx_mult_cmplx_q31, 3, in_com1, in_com2, ref_mult_cmplx,
		     3);
DEFINE_TEST_VARIANT4(complexmath_q31, arm_cmplx_mult_cmplx_q31, 8, in_com1, in_com2, ref_mult_cmplx,
		     8);
DEFINE_TEST_VARIANT4(complexmath_q31, arm_cmplx_mult_cmplx_q31, 11, in_com1, in_com2,
		     ref_mult_cmplx, 11);

static void test_arm_cmplx_mult_real_q31(
	const q31_t *input1, const q31_t *input2, const q31_t *ref,
	size_t length)
{
	size_t buf_length;
	q31_t *output;

	/* Complex number buffer length is twice the data length */
	buf_length = 2 * length;

	/* Allocate output buffer */
	output = malloc(buf_length * sizeof(q31_t));
	zassert_not_null(output, ASSERT_MSG_BUFFER_ALLOC_FAILED);

	/* Run test function */
	arm_cmplx_mult_real_q31(input1, input2, output, length);

	/* Validate output */
	zassert_true(
		test_snr_error_q31(buf_length, output, ref, SNR_ERROR_THRESH),
		ASSERT_MSG_SNR_LIMIT_EXCEED);

	zassert_true(
		test_near_equal_q31(buf_length, output, ref,
			ABS_ERROR_THRESH_Q31),
		ASSERT_MSG_ABS_ERROR_LIMIT_EXCEED);

	/* Free output buffer */
	free(output);
}

DEFINE_TEST_VARIANT4(complexmath_q31, arm_cmplx_mult_real_q31, 3, in_com1, in_com3, ref_mult_real,
		     3);
DEFINE_TEST_VARIANT4(complexmath_q31, arm_cmplx_mult_real_q31, 8, in_com1, in_com3, ref_mult_real,
		     8);
DEFINE_TEST_VARIANT4(complexmath_q31, arm_cmplx_mult_real_q31, 11, in_com1, in_com3, ref_mult_real,
		     11);
