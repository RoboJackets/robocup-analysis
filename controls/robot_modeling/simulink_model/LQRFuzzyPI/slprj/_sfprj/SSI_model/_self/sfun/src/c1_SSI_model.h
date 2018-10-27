#ifndef __c1_SSI_model_h__
#define __c1_SSI_model_h__

/* Include files */
#include "sf_runtime/sfc_sf.h"
#include "sf_runtime/sfc_mex.h"
#include "rtwtypes.h"
#include "multiword_types.h"

/* Type Definitions */
#ifndef typedef_SFc1_SSI_modelInstanceStruct
#define typedef_SFc1_SSI_modelInstanceStruct

typedef struct {
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint32_T chartNumber;
  uint32_T instanceNumber;
  int32_T c1_sfEvent;
  boolean_T c1_doneDoubleBufferReInit;
  uint8_T c1_is_active_c1_SSI_model;
  void *c1_fEmlrtCtx;
  real_T (*c1_start)[3];
  real_T (*c1_target)[3];
  real_T (*c1_y)[3];
  real_T *c1_t_start;
  real_T *c1_t_current;
} SFc1_SSI_modelInstanceStruct;

#endif                                 /*typedef_SFc1_SSI_modelInstanceStruct*/

/* Named Constants */

/* Variable Declarations */
extern struct SfDebugInstanceStruct *sfGlobalDebugInstanceStruct;

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c1_SSI_model_get_eml_resolved_functions_info(void);

/* Function Definitions */
extern void sf_c1_SSI_model_get_check_sum(mxArray *plhs[]);
extern void c1_SSI_model_method_dispatcher(SimStruct *S, int_T method, void
  *data);

#endif
