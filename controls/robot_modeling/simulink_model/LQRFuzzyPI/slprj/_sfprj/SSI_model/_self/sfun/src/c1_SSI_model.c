/* Include files */

#include "SSI_model_sfun.h"
#include "c1_SSI_model.h"
#include "mwmathutil.h"
#define CHARTINSTANCE_CHARTNUMBER      (chartInstance->chartNumber)
#define CHARTINSTANCE_INSTANCENUMBER   (chartInstance->instanceNumber)
#include "SSI_model_sfun_debug_macros.h"
#define _SF_MEX_LISTEN_FOR_CTRL_C(S)   sf_mex_listen_for_ctrl_c_with_debugger(S, sfGlobalDebugInstanceStruct);

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization);
static void chart_debug_initialize_data_addresses(SimStruct *S);
static const mxArray* sf_opaque_get_hover_data_for_msg(void *chartInstance,
  int32_T msgSSID);

/* Type Definitions */

/* Named Constants */
#define CALL_EVENT                     (-1)

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static const char * c1_debug_family_names[14] = { "i", "a_c", "s_c", "t_a",
  "t_s", "t_end", "sign", "nargin", "nargout", "start", "target", "t_start",
  "t_current", "y" };

/* Function Declarations */
static void initialize_c1_SSI_model(SFc1_SSI_modelInstanceStruct *chartInstance);
static void initialize_params_c1_SSI_model(SFc1_SSI_modelInstanceStruct
  *chartInstance);
static void enable_c1_SSI_model(SFc1_SSI_modelInstanceStruct *chartInstance);
static void disable_c1_SSI_model(SFc1_SSI_modelInstanceStruct *chartInstance);
static void c1_update_debugger_state_c1_SSI_model(SFc1_SSI_modelInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c1_SSI_model(SFc1_SSI_modelInstanceStruct
  *chartInstance);
static void set_sim_state_c1_SSI_model(SFc1_SSI_modelInstanceStruct
  *chartInstance, const mxArray *c1_st);
static void finalize_c1_SSI_model(SFc1_SSI_modelInstanceStruct *chartInstance);
static void sf_gateway_c1_SSI_model(SFc1_SSI_modelInstanceStruct *chartInstance);
static void mdl_start_c1_SSI_model(SFc1_SSI_modelInstanceStruct *chartInstance);
static void initSimStructsc1_SSI_model(SFc1_SSI_modelInstanceStruct
  *chartInstance);
static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber);
static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData);
static void c1_emlrt_marshallIn(SFc1_SSI_modelInstanceStruct *chartInstance,
  const mxArray *c1_b_y, const char_T *c1_identifier, real_T c1_c_y[3]);
static void c1_b_emlrt_marshallIn(SFc1_SSI_modelInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_b_y[3]);
static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static real_T c1_c_emlrt_marshallIn(SFc1_SSI_modelInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static real_T c1_mpower(SFc1_SSI_modelInstanceStruct *chartInstance, real_T c1_a);
static void c1_error(SFc1_SSI_modelInstanceStruct *chartInstance);
static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData);
static int32_T c1_d_emlrt_marshallIn(SFc1_SSI_modelInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData);
static uint8_T c1_e_emlrt_marshallIn(SFc1_SSI_modelInstanceStruct *chartInstance,
  const mxArray *c1_b_is_active_c1_SSI_model, const char_T *c1_identifier);
static uint8_T c1_f_emlrt_marshallIn(SFc1_SSI_modelInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId);
static void init_dsm_address_info(SFc1_SSI_modelInstanceStruct *chartInstance);
static void init_simulink_io_address(SFc1_SSI_modelInstanceStruct *chartInstance);

/* Function Definitions */
static void initialize_c1_SSI_model(SFc1_SSI_modelInstanceStruct *chartInstance)
{
  if (sf_is_first_init_cond(chartInstance->S)) {
    initSimStructsc1_SSI_model(chartInstance);
    chart_debug_initialize_data_addresses(chartInstance->S);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c1_is_active_c1_SSI_model = 0U;
}

static void initialize_params_c1_SSI_model(SFc1_SSI_modelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void enable_c1_SSI_model(SFc1_SSI_modelInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c1_SSI_model(SFc1_SSI_modelInstanceStruct *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c1_update_debugger_state_c1_SSI_model(SFc1_SSI_modelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static const mxArray *get_sim_state_c1_SSI_model(SFc1_SSI_modelInstanceStruct
  *chartInstance)
{
  const mxArray *c1_st;
  const mxArray *c1_b_y = NULL;
  const mxArray *c1_c_y = NULL;
  uint8_T c1_hoistedGlobal;
  const mxArray *c1_d_y = NULL;
  c1_st = NULL;
  c1_st = NULL;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_createcellmatrix(2, 1), false);
  c1_c_y = NULL;
  sf_mex_assign(&c1_c_y, sf_mex_create("y", *chartInstance->c1_y, 0, 0U, 1U, 0U,
    1, 3), false);
  sf_mex_setcell(c1_b_y, 0, c1_c_y);
  c1_hoistedGlobal = chartInstance->c1_is_active_c1_SSI_model;
  c1_d_y = NULL;
  sf_mex_assign(&c1_d_y, sf_mex_create("y", &c1_hoistedGlobal, 3, 0U, 0U, 0U, 0),
                false);
  sf_mex_setcell(c1_b_y, 1, c1_d_y);
  sf_mex_assign(&c1_st, c1_b_y, false);
  return c1_st;
}

static void set_sim_state_c1_SSI_model(SFc1_SSI_modelInstanceStruct
  *chartInstance, const mxArray *c1_st)
{
  const mxArray *c1_u;
  real_T c1_dv0[3];
  int32_T c1_i0;
  chartInstance->c1_doneDoubleBufferReInit = true;
  c1_u = sf_mex_dup(c1_st);
  c1_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell("y", c1_u, 0)),
                      "y", c1_dv0);
  for (c1_i0 = 0; c1_i0 < 3; c1_i0++) {
    (*chartInstance->c1_y)[c1_i0] = c1_dv0[c1_i0];
  }

  chartInstance->c1_is_active_c1_SSI_model = c1_e_emlrt_marshallIn(chartInstance,
    sf_mex_dup(sf_mex_getcell("is_active_c1_SSI_model", c1_u, 1)),
    "is_active_c1_SSI_model");
  sf_mex_destroy(&c1_u);
  c1_update_debugger_state_c1_SSI_model(chartInstance);
  sf_mex_destroy(&c1_st);
}

static void finalize_c1_SSI_model(SFc1_SSI_modelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void sf_gateway_c1_SSI_model(SFc1_SSI_modelInstanceStruct *chartInstance)
{
  int32_T c1_i1;
  int32_T c1_i2;
  real_T c1_hoistedGlobal;
  real_T c1_b_hoistedGlobal;
  int32_T c1_i3;
  int32_T c1_i4;
  real_T c1_b_start[3];
  real_T c1_b_t_start;
  real_T c1_b_target[3];
  real_T c1_b_t_current;
  uint32_T c1_debug_family_var_map[14];
  real_T c1_i;
  real_T c1_a_c;
  real_T c1_s_c;
  real_T c1_t_a;
  real_T c1_t_s;
  real_T c1_t_end;
  real_T c1_sign;
  real_T c1_nargin = 4.0;
  real_T c1_nargout = 1.0;
  real_T c1_b_y[3];
  int32_T c1_i5;
  int32_T c1_b_i;
  int32_T c1_i6;
  int32_T c1_i7;
  real_T c1_A;
  real_T c1_x;
  real_T c1_b_x;
  real_T c1_c_y;
  boolean_T c1_b;
  real_T c1_d_y;
  real_T c1_c_x;
  real_T c1_d_x;
  real_T c1_e_x;
  real_T c1_d0;
  real_T c1_b_A;
  real_T c1_f_x;
  real_T c1_g_x;
  real_T c1_e_y;
  real_T c1_c_A;
  real_T c1_h_x;
  real_T c1_i_x;
  real_T c1_f_y;
  _SFD_SYMBOL_SCOPE_PUSH(0U, 0U);
  _sfTime_ = sf_get_time(chartInstance->S);
  _SFD_CC_CALL(CHART_ENTER_SFUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_t_current, 3U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  _SFD_DATA_RANGE_CHECK(*chartInstance->c1_t_start, 2U, 1U, 0U,
                        chartInstance->c1_sfEvent, false);
  for (c1_i1 = 0; c1_i1 < 3; c1_i1++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_target)[c1_i1], 1U, 1U, 0U,
                          chartInstance->c1_sfEvent, false);
  }

  for (c1_i2 = 0; c1_i2 < 3; c1_i2++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_start)[c1_i2], 0U, 1U, 0U,
                          chartInstance->c1_sfEvent, false);
  }

  chartInstance->c1_sfEvent = CALL_EVENT;
  _SFD_CC_CALL(CHART_ENTER_DURING_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  c1_hoistedGlobal = *chartInstance->c1_t_start;
  c1_b_hoistedGlobal = *chartInstance->c1_t_current;
  for (c1_i3 = 0; c1_i3 < 3; c1_i3++) {
    c1_b_start[c1_i3] = (*chartInstance->c1_start)[c1_i3];
  }

  for (c1_i4 = 0; c1_i4 < 3; c1_i4++) {
    c1_b_target[c1_i4] = (*chartInstance->c1_target)[c1_i4];
  }

  c1_b_t_start = c1_hoistedGlobal;
  c1_b_t_current = c1_b_hoistedGlobal;
  _SFD_SYMBOL_SCOPE_PUSH_EML(0U, 14U, 14U, c1_debug_family_names,
    c1_debug_family_var_map);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_i, 0U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_a_c, 1U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_s_c, 2U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_t_a, 3U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_t_s, 4U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_t_end, 5U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_sign, 6U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargin, 7U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(&c1_nargout, 8U, c1_b_sf_marshallOut,
    c1_b_sf_marshallIn);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_b_start, 9U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(c1_b_target, 10U, c1_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_t_start, 11U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML(&c1_b_t_current, 12U, c1_b_sf_marshallOut);
  _SFD_SYMBOL_SCOPE_ADD_EML_IMPORTABLE(c1_b_y, 13U, c1_sf_marshallOut,
    c1_sf_marshallIn);
  CV_EML_FCN(0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 2);
  for (c1_i5 = 0; c1_i5 < 3; c1_i5++) {
    c1_b_y[c1_i5] = 0.0;
  }

  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 3);
  c1_i = 1.0;
  c1_b_i = 0;
  while (c1_b_i < 3) {
    c1_i = 1.0 + (real_T)c1_b_i;
    CV_EML_FOR(0, 1, 0, 1);
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 4);
    c1_a_c = 2.0;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 5);
    c1_s_c = 1.0;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 6);
    c1_t_a = 0.5;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 7);
    c1_A = c1_b_target[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
      chartInstance->S, 1U, 169, 9, MAX_uint32_T, (int32_T)sf_integer_check
      (chartInstance->S, 1U, 169U, 9U, c1_i), 1, 3) - 1] -
      c1_b_start[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
      chartInstance->S, 1U, 181, 8, MAX_uint32_T, (int32_T)sf_integer_check
      (chartInstance->S, 1U, 181U, 8U, c1_i), 1, 3) - 1];
    c1_x = c1_A;
    c1_b_x = c1_x;
    c1_c_y = c1_b_x;
    c1_t_s = c1_c_y - c1_t_a;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 8);
    c1_t_end = ((c1_b_t_start + c1_t_a) + c1_t_s) + c1_t_a;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 10);
    c1_b = (c1_b_target[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
             chartInstance->S, 1U, 256, 9, MAX_uint32_T, (int32_T)
             sf_integer_check(chartInstance->S, 1U, 256U, 9U, c1_i), 1, 3) - 1] >
            c1_b_start[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
             chartInstance->S, 1U, 268, 8, MAX_uint32_T, (int32_T)
             sf_integer_check(chartInstance->S, 1U, 268U, 8U, c1_i), 1, 3) - 1]);
    c1_d_y = 2.0 * (real_T)c1_b;
    c1_sign = c1_d_y - 1.0;
    _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 12);
    c1_c_x = c1_b_start[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
      chartInstance->S, 1U, 296, 8, MAX_uint32_T, (int32_T)sf_integer_check
      (chartInstance->S, 1U, 296U, 8U, c1_i), 1, 3) - 1] -
      c1_b_target[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
      chartInstance->S, 1U, 307, 9, MAX_uint32_T, (int32_T)sf_integer_check
      (chartInstance->S, 1U, 307U, 9U, c1_i), 1, 3) - 1];
    c1_d_x = c1_c_x;
    c1_e_x = c1_d_x;
    c1_d0 = muDoubleScalarAbs(c1_e_x);
    if (CV_EML_IF(0, 1, 0, CV_RELATIONAL_EVAL(4U, 0U, 0, c1_d0, 0.01, -1, 2U,
          c1_d0 < 0.01))) {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 13);
      c1_b_y[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
        chartInstance->S, 1U, 336, 1, MAX_uint32_T, (int32_T)sf_integer_check
        (chartInstance->S, 1U, 336U, 1U, c1_i), 1, 3) - 1] = 0.0;
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 14);
    } else {
      _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 17);
      if (CV_EML_IF(0, 1, 1, CV_RELATIONAL_EVAL(4U, 0U, 1, c1_b_t_current,
            c1_b_t_start, -1, 2U, c1_b_t_current < c1_b_t_start))) {
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 18);
        c1_b_y[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
          chartInstance->S, 1U, 410, 4, MAX_uint32_T, (int32_T)sf_integer_check
          (chartInstance->S, 1U, 410U, 4U, c1_i), 1, 3) - 1] =
          c1_b_start[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
          chartInstance->S, 1U, 417, 8, MAX_uint32_T, (int32_T)sf_integer_check
          (chartInstance->S, 1U, 417U, 8U, c1_i), 1, 3) - 1];
      } else {
        _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 19);
        if (CV_EML_IF(0, 1, 2, CV_RELATIONAL_EVAL(4U, 0U, 2, c1_b_t_current,
              c1_b_t_start + c1_t_a, -1, 2U, c1_b_t_current < c1_b_t_start +
              c1_t_a))) {
          _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 20);
          c1_b_A = c1_sign;
          c1_f_x = c1_b_A;
          c1_g_x = c1_f_x;
          c1_e_y = c1_g_x / 2.0;
          c1_b_y[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
            chartInstance->S, 1U, 472, 4, MAX_uint32_T, (int32_T)
            sf_integer_check(chartInstance->S, 1U, 472U, 4U, c1_i), 1, 3) - 1] =
            c1_b_start[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
            chartInstance->S, 1U, 479, 8, MAX_uint32_T, (int32_T)
            sf_integer_check(chartInstance->S, 1U, 479U, 8U, c1_i), 1, 3) - 1] +
            c1_e_y * c1_a_c * c1_mpower(chartInstance, c1_b_t_current -
            c1_b_t_start);
        } else {
          _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 21);
          if (CV_EML_IF(0, 1, 3, CV_RELATIONAL_EVAL(4U, 0U, 3, c1_b_t_current,
                c1_t_end - c1_t_a, -1, 2U, c1_b_t_current < c1_t_end - c1_t_a)))
          {
            _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 22);
            c1_b_y[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
              chartInstance->S, 1U, 571, 4, MAX_uint32_T, (int32_T)
              sf_integer_check(chartInstance->S, 1U, 571U, 4U, c1_i), 1, 3) - 1]
              = 0.5 * (c1_b_start[sf_eml_array_bounds_check
                       (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 583,
                        8, MAX_uint32_T, (int32_T)sf_integer_check
                        (chartInstance->S, 1U, 583U, 8U, c1_i), 1, 3) - 1] +
                       c1_b_target[sf_eml_array_bounds_check
                       (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 594,
                        9, MAX_uint32_T, (int32_T)sf_integer_check
                        (chartInstance->S, 1U, 594U, 9U, c1_i), 1, 3) - 1]) +
              c1_sign * c1_s_c * (c1_b_t_current - 0.5 * (c1_b_t_start +
              c1_t_end));
          } else {
            _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 23);
            if (CV_EML_IF(0, 1, 4, CV_RELATIONAL_EVAL(4U, 0U, 4, c1_b_t_current,
                  c1_t_end, -1, 2U, c1_b_t_current < c1_t_end))) {
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 24);
              c1_c_A = c1_sign;
              c1_h_x = c1_c_A;
              c1_i_x = c1_h_x;
              c1_f_y = c1_i_x / 2.0;
              c1_b_y[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
                chartInstance->S, 1U, 690, 4, MAX_uint32_T, (int32_T)
                sf_integer_check(chartInstance->S, 1U, 690U, 4U, c1_i), 1, 3) -
                1] = c1_b_target[sf_eml_array_bounds_check
                (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 697, 9,
                 MAX_uint32_T, (int32_T)sf_integer_check(chartInstance->S, 1U,
                  697U, 9U, c1_i), 1, 3) - 1] - c1_f_y * c1_a_c * c1_mpower
                (chartInstance, c1_t_end - c1_b_t_current);
            } else {
              _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, 26);
              c1_b_y[sf_eml_array_bounds_check(sfGlobalDebugInstanceStruct,
                chartInstance->S, 1U, 782, 4, MAX_uint32_T, (int32_T)
                sf_integer_check(chartInstance->S, 1U, 782U, 4U, c1_i), 1, 3) -
                1] = c1_b_target[sf_eml_array_bounds_check
                (sfGlobalDebugInstanceStruct, chartInstance->S, 1U, 789, 9,
                 MAX_uint32_T, (int32_T)sf_integer_check(chartInstance->S, 1U,
                  789U, 9U, c1_i), 1, 3) - 1];
            }
          }
        }
      }
    }

    c1_b_i++;
    _SF_MEX_LISTEN_FOR_CTRL_C(chartInstance->S);
  }

  CV_EML_FOR(0, 1, 0, 0);
  _SFD_EML_CALL(0U, chartInstance->c1_sfEvent, -26);
  _SFD_SYMBOL_SCOPE_POP();
  for (c1_i6 = 0; c1_i6 < 3; c1_i6++) {
    (*chartInstance->c1_y)[c1_i6] = c1_b_y[c1_i6];
  }

  _SFD_CC_CALL(EXIT_OUT_OF_FUNCTION_TAG, 0U, chartInstance->c1_sfEvent);
  _SFD_SYMBOL_SCOPE_POP();
  _SFD_CHECK_FOR_STATE_INCONSISTENCY(_SSI_modelMachineNumber_,
    chartInstance->chartNumber, chartInstance->instanceNumber);
  for (c1_i7 = 0; c1_i7 < 3; c1_i7++) {
    _SFD_DATA_RANGE_CHECK((*chartInstance->c1_y)[c1_i7], 4U, 1U, 0U,
                          chartInstance->c1_sfEvent, false);
  }
}

static void mdl_start_c1_SSI_model(SFc1_SSI_modelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void initSimStructsc1_SSI_model(SFc1_SSI_modelInstanceStruct
  *chartInstance)
{
  (void)chartInstance;
}

static void init_script_number_translation(uint32_T c1_machineNumber, uint32_T
  c1_chartNumber, uint32_T c1_instanceNumber)
{
  (void)(c1_machineNumber);
  (void)(c1_chartNumber);
  (void)(c1_instanceNumber);
}

static const mxArray *c1_sf_marshallOut(void *chartInstanceVoid, void *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  int32_T c1_i8;
  const mxArray *c1_b_y = NULL;
  real_T c1_u[3];
  SFc1_SSI_modelInstanceStruct *chartInstance;
  chartInstance = (SFc1_SSI_modelInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  for (c1_i8 = 0; c1_i8 < 3; c1_i8++) {
    c1_u[c1_i8] = (*(real_T (*)[3])c1_inData)[c1_i8];
  }

  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_u, 0, 0U, 1U, 0U, 1, 3), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_b_y, false);
  return c1_mxArrayOutData;
}

static void c1_emlrt_marshallIn(SFc1_SSI_modelInstanceStruct *chartInstance,
  const mxArray *c1_b_y, const char_T *c1_identifier, real_T c1_c_y[3])
{
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = (const char *)c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_y), &c1_thisId, c1_c_y);
  sf_mex_destroy(&c1_b_y);
}

static void c1_b_emlrt_marshallIn(SFc1_SSI_modelInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId, real_T c1_b_y[3])
{
  real_T c1_dv1[3];
  int32_T c1_i9;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), c1_dv1, 1, 0, 0U, 1, 0U, 1, 3);
  for (c1_i9 = 0; c1_i9 < 3; c1_i9++) {
    c1_b_y[c1_i9] = c1_dv1[c1_i9];
  }

  sf_mex_destroy(&c1_u);
}

static void c1_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_y;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_c_y[3];
  int32_T c1_i10;
  SFc1_SSI_modelInstanceStruct *chartInstance;
  chartInstance = (SFc1_SSI_modelInstanceStruct *)chartInstanceVoid;
  c1_b_y = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = (const char *)c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_y), &c1_thisId, c1_c_y);
  sf_mex_destroy(&c1_b_y);
  for (c1_i10 = 0; c1_i10 < 3; c1_i10++) {
    (*(real_T (*)[3])c1_outData)[c1_i10] = c1_c_y[c1_i10];
  }

  sf_mex_destroy(&c1_mxArrayInData);
}

static const mxArray *c1_b_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  real_T c1_u;
  const mxArray *c1_b_y = NULL;
  SFc1_SSI_modelInstanceStruct *chartInstance;
  chartInstance = (SFc1_SSI_modelInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(real_T *)c1_inData;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 0, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_b_y, false);
  return c1_mxArrayOutData;
}

static real_T c1_c_emlrt_marshallIn(SFc1_SSI_modelInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  real_T c1_b_y;
  real_T c1_d1;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_d1, 1, 0, 0U, 0, 0U, 0);
  c1_b_y = c1_d1;
  sf_mex_destroy(&c1_u);
  return c1_b_y;
}

static void c1_b_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_nargout;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  real_T c1_b_y;
  SFc1_SSI_modelInstanceStruct *chartInstance;
  chartInstance = (SFc1_SSI_modelInstanceStruct *)chartInstanceVoid;
  c1_nargout = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = (const char *)c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_b_y = c1_c_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_nargout),
    &c1_thisId);
  sf_mex_destroy(&c1_nargout);
  *(real_T *)c1_outData = c1_b_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

const mxArray *sf_c1_SSI_model_get_eml_resolved_functions_info(void)
{
  const mxArray *c1_nameCaptureInfo = NULL;
  c1_nameCaptureInfo = NULL;
  sf_mex_assign(&c1_nameCaptureInfo, sf_mex_create("nameCaptureInfo", NULL, 0,
    0U, 1U, 0U, 2, 0, 1), false);
  return c1_nameCaptureInfo;
}

static real_T c1_mpower(SFc1_SSI_modelInstanceStruct *chartInstance, real_T c1_a)
{
  real_T c1_c;
  real_T c1_b_a;
  real_T c1_c_a;
  real_T c1_x;
  real_T c1_d_a;
  boolean_T c1_p;
  c1_b_a = c1_a;
  c1_c_a = c1_b_a;
  c1_x = c1_c_a;
  c1_d_a = c1_x;
  c1_c = c1_d_a * c1_d_a;
  c1_p = false;
  if (c1_p) {
    c1_error(chartInstance);
  }

  return c1_c;
}

static void c1_error(SFc1_SSI_modelInstanceStruct *chartInstance)
{
  const mxArray *c1_b_y = NULL;
  static char_T c1_cv0[31] = { 'C', 'o', 'd', 'e', 'r', ':', 't', 'o', 'o', 'l',
    'b', 'o', 'x', ':', 'p', 'o', 'w', 'e', 'r', '_', 'd', 'o', 'm', 'a', 'i',
    'n', 'E', 'r', 'r', 'o', 'r' };

  (void)chartInstance;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", c1_cv0, 10, 0U, 1U, 0U, 2, 1, 31),
                false);
  sf_mex_call_debug(sfGlobalDebugInstanceStruct, "error", 0U, 1U, 14,
                    sf_mex_call_debug(sfGlobalDebugInstanceStruct, "message", 1U,
    1U, 14, c1_b_y));
}

static const mxArray *c1_c_sf_marshallOut(void *chartInstanceVoid, void
  *c1_inData)
{
  const mxArray *c1_mxArrayOutData;
  int32_T c1_u;
  const mxArray *c1_b_y = NULL;
  SFc1_SSI_modelInstanceStruct *chartInstance;
  chartInstance = (SFc1_SSI_modelInstanceStruct *)chartInstanceVoid;
  c1_mxArrayOutData = NULL;
  c1_mxArrayOutData = NULL;
  c1_u = *(int32_T *)c1_inData;
  c1_b_y = NULL;
  sf_mex_assign(&c1_b_y, sf_mex_create("y", &c1_u, 6, 0U, 0U, 0U, 0), false);
  sf_mex_assign(&c1_mxArrayOutData, c1_b_y, false);
  return c1_mxArrayOutData;
}

static int32_T c1_d_emlrt_marshallIn(SFc1_SSI_modelInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  int32_T c1_b_y;
  int32_T c1_i11;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_i11, 1, 6, 0U, 0, 0U, 0);
  c1_b_y = c1_i11;
  sf_mex_destroy(&c1_u);
  return c1_b_y;
}

static void c1_c_sf_marshallIn(void *chartInstanceVoid, const mxArray
  *c1_mxArrayInData, const char_T *c1_varName, void *c1_outData)
{
  const mxArray *c1_b_sfEvent;
  const char_T *c1_identifier;
  emlrtMsgIdentifier c1_thisId;
  int32_T c1_b_y;
  SFc1_SSI_modelInstanceStruct *chartInstance;
  chartInstance = (SFc1_SSI_modelInstanceStruct *)chartInstanceVoid;
  c1_b_sfEvent = sf_mex_dup(c1_mxArrayInData);
  c1_identifier = c1_varName;
  c1_thisId.fIdentifier = (const char *)c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_b_y = c1_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c1_b_sfEvent),
    &c1_thisId);
  sf_mex_destroy(&c1_b_sfEvent);
  *(int32_T *)c1_outData = c1_b_y;
  sf_mex_destroy(&c1_mxArrayInData);
}

static uint8_T c1_e_emlrt_marshallIn(SFc1_SSI_modelInstanceStruct *chartInstance,
  const mxArray *c1_b_is_active_c1_SSI_model, const char_T *c1_identifier)
{
  uint8_T c1_b_y;
  emlrtMsgIdentifier c1_thisId;
  c1_thisId.fIdentifier = (const char *)c1_identifier;
  c1_thisId.fParent = NULL;
  c1_thisId.bParentIsCell = false;
  c1_b_y = c1_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c1_b_is_active_c1_SSI_model), &c1_thisId);
  sf_mex_destroy(&c1_b_is_active_c1_SSI_model);
  return c1_b_y;
}

static uint8_T c1_f_emlrt_marshallIn(SFc1_SSI_modelInstanceStruct *chartInstance,
  const mxArray *c1_u, const emlrtMsgIdentifier *c1_parentId)
{
  uint8_T c1_b_y;
  uint8_T c1_u0;
  (void)chartInstance;
  sf_mex_import(c1_parentId, sf_mex_dup(c1_u), &c1_u0, 1, 3, 0U, 0, 0U, 0);
  c1_b_y = c1_u0;
  sf_mex_destroy(&c1_u);
  return c1_b_y;
}

static void init_dsm_address_info(SFc1_SSI_modelInstanceStruct *chartInstance)
{
  (void)chartInstance;
}

static void init_simulink_io_address(SFc1_SSI_modelInstanceStruct *chartInstance)
{
  chartInstance->c1_fEmlrtCtx = (void *)sfrtGetEmlrtCtx(chartInstance->S);
  chartInstance->c1_start = (real_T (*)[3])ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c1_target = (real_T (*)[3])ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c1_y = (real_T (*)[3])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c1_t_start = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c1_t_current = (real_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
}

/* SFunction Glue Code */
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

void sf_c1_SSI_model_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3521533251U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(1226303221U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(200786848U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(1697078948U);
}

mxArray* sf_c1_SSI_model_get_post_codegen_info(void);
mxArray *sf_c1_SSI_model_get_autoinheritance_info(void)
{
  const char *autoinheritanceFields[] = { "checksum", "inputs", "parameters",
    "outputs", "locals", "postCodegenInfo" };

  mxArray *mxAutoinheritanceInfo = mxCreateStructMatrix(1, 1, sizeof
    (autoinheritanceFields)/sizeof(autoinheritanceFields[0]),
    autoinheritanceFields);

  {
    mxArray *mxChecksum = mxCreateString("jtnRlY87Z1W2ak4pQftceC");
    mxSetField(mxAutoinheritanceInfo,0,"checksum",mxChecksum);
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,4,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      mxSetField(mxData,1,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,1,"type",mxType);
    }

    mxSetField(mxData,1,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,2,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,2,"type",mxType);
    }

    mxSetField(mxData,2,"complexity",mxCreateDoubleScalar(0));

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,0,mxREAL);
      double *pr = mxGetPr(mxSize);
      mxSetField(mxData,3,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,3,"type",mxType);
    }

    mxSetField(mxData,3,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"inputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"parameters",mxCreateDoubleMatrix(0,0,
                mxREAL));
  }

  {
    const char *dataFields[] = { "size", "type", "complexity" };

    mxArray *mxData = mxCreateStructMatrix(1,1,3,dataFields);

    {
      mxArray *mxSize = mxCreateDoubleMatrix(1,1,mxREAL);
      double *pr = mxGetPr(mxSize);
      pr[0] = (double)(3);
      mxSetField(mxData,0,"size",mxSize);
    }

    {
      const char *typeFields[] = { "base", "fixpt", "isFixedPointType" };

      mxArray *mxType = mxCreateStructMatrix(1,1,sizeof(typeFields)/sizeof
        (typeFields[0]),typeFields);
      mxSetField(mxType,0,"base",mxCreateDoubleScalar(10));
      mxSetField(mxType,0,"fixpt",mxCreateDoubleMatrix(0,0,mxREAL));
      mxSetField(mxType,0,"isFixedPointType",mxCreateDoubleScalar(0));
      mxSetField(mxData,0,"type",mxType);
    }

    mxSetField(mxData,0,"complexity",mxCreateDoubleScalar(0));
    mxSetField(mxAutoinheritanceInfo,0,"outputs",mxData);
  }

  {
    mxSetField(mxAutoinheritanceInfo,0,"locals",mxCreateDoubleMatrix(0,0,mxREAL));
  }

  {
    mxArray* mxPostCodegenInfo = sf_c1_SSI_model_get_post_codegen_info();
    mxSetField(mxAutoinheritanceInfo,0,"postCodegenInfo",mxPostCodegenInfo);
  }

  return(mxAutoinheritanceInfo);
}

mxArray *sf_c1_SSI_model_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c1_SSI_model_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("hasBreakpoints");
  mxArray *hiddenFallbackType = mxCreateString("none");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c1_SSI_model_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = mxCreateCellMatrix(1,0);
  return mxBIArgs;
}

mxArray* sf_c1_SSI_model_get_post_codegen_info(void)
{
  const char* fieldNames[] = { "exportedFunctionsUsedByThisChart",
    "exportedFunctionsChecksum" };

  mwSize dims[2] = { 1, 1 };

  mxArray* mxPostCodegenInfo = mxCreateStructArray(2, dims, sizeof(fieldNames)/
    sizeof(fieldNames[0]), fieldNames);

  {
    mxArray* mxExportedFunctionsChecksum = mxCreateString("");
    mwSize exp_dims[2] = { 0, 1 };

    mxArray* mxExportedFunctionsUsedByThisChart = mxCreateCellArray(2, exp_dims);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsUsedByThisChart",
               mxExportedFunctionsUsedByThisChart);
    mxSetField(mxPostCodegenInfo, 0, "exportedFunctionsChecksum",
               mxExportedFunctionsChecksum);
  }

  return mxPostCodegenInfo;
}

static const mxArray *sf_get_sim_state_info_c1_SSI_model(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  const char *infoEncStr[] = {
    "100 S1x2'type','srcId','name','auxInfo'{{M[1],M[5],T\"y\",},{M[8],M[0],T\"is_active_c1_SSI_model\",}}"
  };

  mxArray *mxVarInfo = sf_mex_decode_encoded_mx_struct_array(infoEncStr, 2, 10);
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c1_SSI_model_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static void chart_debug_initialization(SimStruct *S, unsigned int
  fullDebuggerInitialization)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_SSI_modelInstanceStruct *chartInstance = (SFc1_SSI_modelInstanceStruct *)
      sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S) && fullDebuggerInitialization==1) {
      /* do this only if simulation is starting */
      {
        unsigned int chartAlreadyPresent;
        chartAlreadyPresent = sf_debug_initialize_chart
          (sfGlobalDebugInstanceStruct,
           _SSI_modelMachineNumber_,
           1,
           1,
           1,
           0,
           5,
           0,
           0,
           0,
           0,
           0,
           &chartInstance->chartNumber,
           &chartInstance->instanceNumber,
           (void *)S);

        /* Each instance must initialize its own list of scripts */
        init_script_number_translation(_SSI_modelMachineNumber_,
          chartInstance->chartNumber,chartInstance->instanceNumber);
        if (chartAlreadyPresent==0) {
          /* this is the first instance */
          sf_debug_set_chart_disable_implicit_casting
            (sfGlobalDebugInstanceStruct,_SSI_modelMachineNumber_,
             chartInstance->chartNumber,1);
          sf_debug_set_chart_event_thresholds(sfGlobalDebugInstanceStruct,
            _SSI_modelMachineNumber_,
            chartInstance->chartNumber,
            0,
            0,
            0);
          _SFD_SET_DATA_PROPS(0,1,1,0,"start");
          _SFD_SET_DATA_PROPS(1,1,1,0,"target");
          _SFD_SET_DATA_PROPS(2,1,1,0,"t_start");
          _SFD_SET_DATA_PROPS(3,1,1,0,"t_current");
          _SFD_SET_DATA_PROPS(4,2,0,1,"y");
          _SFD_STATE_INFO(0,0,2);
          _SFD_CH_SUBSTATE_COUNT(0);
          _SFD_CH_SUBSTATE_DECOMP(0);
        }

        _SFD_CV_INIT_CHART(0,0,0,0);

        {
          _SFD_CV_INIT_STATE(0,0,0,0,0,0,NULL,NULL);
        }

        _SFD_CV_INIT_TRANS(0,0,NULL,NULL,0,NULL);

        /* Initialization of MATLAB Function Model Coverage */
        _SFD_CV_INIT_EML(0,1,1,0,5,0,0,0,1,0,0,0);
        _SFD_CV_INIT_EML_FCN(0,0,"eML_blk_kernel",0,-1,811);
        _SFD_CV_INIT_EML_IF(0,1,0,288,325,379,807);
        _SFD_CV_INIT_EML_IF(0,1,1,379,401,431,807);
        _SFD_CV_INIT_EML_IF(0,1,2,431,463,532,807);
        _SFD_CV_INIT_EML_IF(0,1,3,532,562,657,807);
        _SFD_CV_INIT_EML_IF(0,1,4,657,681,749,807);
        _SFD_CV_INIT_EML_FOR(0,1,0,75,87,811);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,0,292,324,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,1,382,401,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,2,438,463,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,3,539,562,-1,2);
        _SFD_CV_INIT_EML_RELATIONAL(0,1,4,664,681,-1,2);

        {
          unsigned int dimVector[1];
          dimVector[0]= 3U;
          _SFD_SET_DATA_COMPILED_PROPS(0,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        }

        {
          unsigned int dimVector[1];
          dimVector[0]= 3U;
          _SFD_SET_DATA_COMPILED_PROPS(1,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)NULL);
        }

        _SFD_SET_DATA_COMPILED_PROPS(2,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);
        _SFD_SET_DATA_COMPILED_PROPS(3,SF_DOUBLE,0,NULL,0,0,0,0.0,1.0,0,0,
          (MexFcnForType)c1_b_sf_marshallOut,(MexInFcnForType)NULL);

        {
          unsigned int dimVector[1];
          dimVector[0]= 3U;
          _SFD_SET_DATA_COMPILED_PROPS(4,SF_DOUBLE,1,&(dimVector[0]),0,0,0,0.0,
            1.0,0,0,(MexFcnForType)c1_sf_marshallOut,(MexInFcnForType)
            c1_sf_marshallIn);
        }
      }
    } else {
      sf_debug_reset_current_state_configuration(sfGlobalDebugInstanceStruct,
        _SSI_modelMachineNumber_,chartInstance->chartNumber,
        chartInstance->instanceNumber);
    }
  }
}

static void chart_debug_initialize_data_addresses(SimStruct *S)
{
  if (!sim_mode_is_rtw_gen(S)) {
    SFc1_SSI_modelInstanceStruct *chartInstance = (SFc1_SSI_modelInstanceStruct *)
      sf_get_chart_instance_ptr(S);
    if (ssIsFirstInitCond(S)) {
      /* do this only if simulation is starting and after we know the addresses of all data */
      {
        _SFD_SET_DATA_VALUE_PTR(0U, (void *)chartInstance->c1_start);
        _SFD_SET_DATA_VALUE_PTR(1U, (void *)chartInstance->c1_target);
        _SFD_SET_DATA_VALUE_PTR(4U, (void *)chartInstance->c1_y);
        _SFD_SET_DATA_VALUE_PTR(2U, (void *)chartInstance->c1_t_start);
        _SFD_SET_DATA_VALUE_PTR(3U, (void *)chartInstance->c1_t_current);
      }
    }
  }
}

static const char* sf_get_instance_specialization(void)
{
  return "smxOE6Ay3EHucCRKHpfQNd";
}

static void sf_opaque_initialize_c1_SSI_model(void *chartInstanceVar)
{
  chart_debug_initialization(((SFc1_SSI_modelInstanceStruct*) chartInstanceVar
    )->S,0);
  initialize_params_c1_SSI_model((SFc1_SSI_modelInstanceStruct*)
    chartInstanceVar);
  initialize_c1_SSI_model((SFc1_SSI_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_enable_c1_SSI_model(void *chartInstanceVar)
{
  enable_c1_SSI_model((SFc1_SSI_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c1_SSI_model(void *chartInstanceVar)
{
  disable_c1_SSI_model((SFc1_SSI_modelInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c1_SSI_model(void *chartInstanceVar)
{
  sf_gateway_c1_SSI_model((SFc1_SSI_modelInstanceStruct*) chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c1_SSI_model(SimStruct* S)
{
  return get_sim_state_c1_SSI_model((SFc1_SSI_modelInstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c1_SSI_model(SimStruct* S, const mxArray *st)
{
  set_sim_state_c1_SSI_model((SFc1_SSI_modelInstanceStruct*)
    sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_terminate_c1_SSI_model(void *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc1_SSI_modelInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_SSI_model_optimization_info();
    }

    finalize_c1_SSI_model((SFc1_SSI_modelInstanceStruct*) chartInstanceVar);
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc1_SSI_model((SFc1_SSI_modelInstanceStruct*) chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c1_SSI_model(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  if (sf_machine_global_initializer_called()) {
    initialize_params_c1_SSI_model((SFc1_SSI_modelInstanceStruct*)
      sf_get_chart_instance_ptr(S));
  }
}

static void mdlSetWorkWidths_c1_SSI_model(SimStruct *S)
{
  /* Set overwritable ports for inplace optimization */
  ssSetStatesModifiedOnlyInUpdate(S, 1);
  ssMdlUpdateIsEmpty(S, 1);
  if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
    mxArray *infoStruct = load_SSI_model_optimization_info(sim_mode_is_rtw_gen(S),
      sim_mode_is_modelref_sim(S), sim_mode_is_external(S));
    int_T chartIsInlinable =
      (int_T)sf_is_chart_inlinable(sf_get_instance_specialization(),infoStruct,1);
    ssSetStateflowIsInlinable(S,chartIsInlinable);
    ssSetRTWCG(S,1);
    ssSetEnableFcnIsTrivial(S,1);
    ssSetDisableFcnIsTrivial(S,1);
    ssSetNotMultipleInlinable(S,sf_rtw_info_uint_prop
      (sf_get_instance_specialization(),infoStruct,1,
       "gatewayCannotBeInlinedMultipleTimes"));
    sf_set_chart_accesses_machine_info(S, sf_get_instance_specialization(),
      infoStruct, 1);
    sf_update_buildInfo(S, sf_get_instance_specialization(),infoStruct,1);
    if (chartIsInlinable) {
      ssSetInputPortOptimOpts(S, 0, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 1, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 2, SS_REUSABLE_AND_LOCAL);
      ssSetInputPortOptimOpts(S, 3, SS_REUSABLE_AND_LOCAL);
      sf_mark_chart_expressionable_inputs(S,sf_get_instance_specialization(),
        infoStruct,1,4);
      sf_mark_chart_reusable_outputs(S,sf_get_instance_specialization(),
        infoStruct,1,1);
    }

    {
      unsigned int outPortIdx;
      for (outPortIdx=1; outPortIdx<=1; ++outPortIdx) {
        ssSetOutputPortOptimizeInIR(S, outPortIdx, 1U);
      }
    }

    {
      unsigned int inPortIdx;
      for (inPortIdx=0; inPortIdx < 4; ++inPortIdx) {
        ssSetInputPortOptimizeInIR(S, inPortIdx, 1U);
      }
    }

    sf_set_rtw_dwork_info(S,sf_get_instance_specialization(),infoStruct,1);
    sf_register_codegen_names_for_scoped_functions_defined_by_chart(S);
    ssSetHasSubFunctions(S,!(chartIsInlinable));
  } else {
  }

  ssSetOptions(S,ssGetOptions(S)|SS_OPTION_WORKS_WITH_CODE_REUSE);
  ssSetChecksum0(S,(4000844464U));
  ssSetChecksum1(S,(1999261152U));
  ssSetChecksum2(S,(2275060925U));
  ssSetChecksum3(S,(168000716U));
  ssSetmdlDerivatives(S, NULL);
  ssSetExplicitFCSSCtrl(S,1);
  ssSetStateSemanticsClassicAndSynchronous(S, true);
  ssSupportsMultipleExecInstances(S,1);
}

static void mdlRTW_c1_SSI_model(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlStart_c1_SSI_model(SimStruct *S)
{
  SFc1_SSI_modelInstanceStruct *chartInstance;
  chartInstance = (SFc1_SSI_modelInstanceStruct *)utMalloc(sizeof
    (SFc1_SSI_modelInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc1_SSI_modelInstanceStruct));
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c1_SSI_model;
  chartInstance->chartInfo.initializeChart = sf_opaque_initialize_c1_SSI_model;
  chartInstance->chartInfo.terminateChart = sf_opaque_terminate_c1_SSI_model;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c1_SSI_model;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c1_SSI_model;
  chartInstance->chartInfo.getSimState = sf_opaque_get_sim_state_c1_SSI_model;
  chartInstance->chartInfo.setSimState = sf_opaque_set_sim_state_c1_SSI_model;
  chartInstance->chartInfo.getSimStateInfo = sf_get_sim_state_info_c1_SSI_model;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c1_SSI_model;
  chartInstance->chartInfo.mdlStart = mdlStart_c1_SSI_model;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c1_SSI_model;
  chartInstance->chartInfo.callGetHoverDataForMsg = NULL;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.debugInstance = sfGlobalDebugInstanceStruct;
  chartInstance->S = S;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  chart_debug_initialization(S,1);
  mdl_start_c1_SSI_model(chartInstance);
}

void c1_SSI_model_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_START:
    mdlStart_c1_SSI_model(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c1_SSI_model(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c1_SSI_model(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c1_SSI_model_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
