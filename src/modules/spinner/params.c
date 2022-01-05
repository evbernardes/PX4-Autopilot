/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file params.c
 *
 * Parameters for spinner controller
 */

/* controller parameters, use max. 15 characters for param name! */

/**
 *
 */
PARAM_DEFINE_FLOAT(SPIN_KP, 1.0f);

/**
 *
 */
PARAM_DEFINE_FLOAT(SPIN_KD, 0.1f);

/**
 *
 */
PARAM_DEFINE_FLOAT(SPIN_KS, 0.1f);

/**
 *
 */
PARAM_DEFINE_FLOAT(SPIN_KC, 0.1f);

/**
 *
 */
PARAM_DEFINE_FLOAT(SPIN_KI, 0.0f);

/**
 *
 */
PARAM_DEFINE_FLOAT(SPIN_ANG_MAX_DEG, 8.0f);

/**
 *
 */
PARAM_DEFINE_FLOAT(SPIN_JXX, 2.7209399e+00);
/**
 *
 */
PARAM_DEFINE_FLOAT(SPIN_JYY, 2.8050915e+00);
/**
 *
 */
PARAM_DEFINE_FLOAT(SPIN_JZZ, 2.5446860e-01);
/**
 *
 */
PARAM_DEFINE_FLOAT(PROP_JZZ, 1.3747044e-02);

/**
 *
 */
PARAM_DEFINE_FLOAT(PROP_HEIGHT, 0.145303f);
/**
 *
 */
PARAM_DEFINE_FLOAT(PROP_K_THRUST, 6.24e-06);
/**
 *
 */
PARAM_DEFINE_FLOAT(PROP_K_TORQUE, -1.08e-08);
/**
 *
 */
PARAM_DEFINE_FLOAT(PROP_K_RATIO, -0.00173);
/**
 *
 */
PARAM_DEFINE_FLOAT(PROP_MAX_INC, 0.175);
/**
 *
 */
PARAM_DEFINE_FLOAT(MAX_ALPHA_DEG, 10.0);
/**
 *
 */
PARAM_DEFINE_FLOAT(MAX_INCLINATION, 0.261799);
/**
 *
 */
PARAM_DEFINE_FLOAT(ANG_VEL_RATIO, 0.02);
