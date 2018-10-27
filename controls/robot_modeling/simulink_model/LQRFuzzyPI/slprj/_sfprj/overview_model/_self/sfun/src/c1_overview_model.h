#ifndef __c1_overview_model_h__
#define __c1_overview_model_h__

/* Type Definitions */
#ifndef typedef_SFc1_overview_modelInstanceStruct
#define typedef_SFc1_overview_modelInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_overview_model;
  void *c1_fEmlrtCtx;
  real_T *c1_x_dot;
  real_T *c1_y_dot;
  real_T *c1_omega_dot;
  real_T (*c1_E)[4];
  real_T (*c1_G)[12];
  real_T *c1_r;
  real_T *c1_s;
} SFc1_overview_modelInstanceStruct;

#endif                                 /*typedef_SFc1_overview_modelInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_overview_model_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_overview_model_get_check_sum(mxArray *plhs[]);
extern void c1_overview_model_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
