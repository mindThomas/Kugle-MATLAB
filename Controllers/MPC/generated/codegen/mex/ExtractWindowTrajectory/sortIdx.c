/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * sortIdx.c
 *
 * Code generation for function 'sortIdx'
 *
 */

/* Include files */
#include "mwmathutil.h"
#include "rt_nonfinite.h"
#include "ExtractWindowTrajectory.h"
#include "sortIdx.h"
#include "ExtractWindowTrajectory_emxutil.h"
#include "eml_int_forloop_overflow_check.h"
#include "ExtractWindowTrajectory_data.h"

/* Variable Definitions */
static emlrtRSInfo jc_emlrtRSI = { 105,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRSInfo kc_emlrtRSI = { 308,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRSInfo lc_emlrtRSI = { 316,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRSInfo mc_emlrtRSI = { 317,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRSInfo nc_emlrtRSI = { 325,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRSInfo oc_emlrtRSI = { 333,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRSInfo pc_emlrtRSI = { 392,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRSInfo qc_emlrtRSI = { 420,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRSInfo rc_emlrtRSI = { 427,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRSInfo sc_emlrtRSI = { 587,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRSInfo tc_emlrtRSI = { 589,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRSInfo uc_emlrtRSI = { 617,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRSInfo vc_emlrtRSI = { 499,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRSInfo xc_emlrtRSI = { 507,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRSInfo yc_emlrtRSI = { 514,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRSInfo ad_emlrtRSI = { 561,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRSInfo bd_emlrtRSI = { 530,/* lineNo */
  "sortIdx",                           /* fcnName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pathName */
};

static emlrtRTEInfo w_emlrtRTEI = { 1, /* lineNo */
  20,                                  /* colNo */
  "sortIdx",                           /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pName */
};

static emlrtRTEInfo ab_emlrtRTEI = { 308,/* lineNo */
  14,                                  /* colNo */
  "sortIdx",                           /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pName */
};

static emlrtRTEInfo bb_emlrtRTEI = { 308,/* lineNo */
  20,                                  /* colNo */
  "sortIdx",                           /* fName */
  "C:\\Program Files\\MATLAB\\R2018a\\toolbox\\eml\\eml\\+coder\\+internal\\sortIdx.m"/* pName */
};

/* Function Declarations */
static void merge(const emlrtStack *sp, emxArray_int32_T *idx, emxArray_real_T
                  *x, int32_T offset, int32_T np, int32_T nq, emxArray_int32_T
                  *iwork, emxArray_real_T *xwork);
static void merge_block(const emlrtStack *sp, emxArray_int32_T *idx,
  emxArray_real_T *x, int32_T offset, int32_T n, int32_T preSortLevel,
  emxArray_int32_T *iwork, emxArray_real_T *xwork);

/* Function Definitions */
static void merge(const emlrtStack *sp, emxArray_int32_T *idx, emxArray_real_T
                  *x, int32_T offset, int32_T np, int32_T nq, emxArray_int32_T
                  *iwork, emxArray_real_T *xwork)
{
  int32_T n;
  int32_T qend;
  int32_T p;
  int32_T iout;
  int32_T exitg1;
  emlrtStack st;
  emlrtStack b_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (nq != 0) {
    n = np + nq;
    st.site = &bd_emlrtRSI;
    if ((!(1 > n)) && (n > 2147483646)) {
      b_st.site = &x_emlrtRSI;
      check_forloop_overflow_error(&b_st);
    }

    for (qend = 0; qend < n; qend++) {
      iwork->data[qend] = idx->data[offset + qend];
      xwork->data[qend] = x->data[offset + qend];
    }

    p = 0;
    n = np;
    qend = np + nq;
    iout = offset - 1;
    do {
      exitg1 = 0;
      iout++;
      if (xwork->data[p] <= xwork->data[n]) {
        idx->data[iout] = iwork->data[p];
        x->data[iout] = xwork->data[p];
        if (p + 1 < np) {
          p++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx->data[iout] = iwork->data[n];
        x->data[iout] = xwork->data[n];
        if (n + 1 < qend) {
          n++;
        } else {
          n = (iout - p) + 1;
          st.site = &ad_emlrtRSI;
          if ((!(p + 1 > np)) && (np > 2147483646)) {
            b_st.site = &x_emlrtRSI;
            check_forloop_overflow_error(&b_st);
          }

          while (p + 1 <= np) {
            idx->data[n + p] = iwork->data[p];
            x->data[n + p] = xwork->data[p];
            p++;
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

static void merge_block(const emlrtStack *sp, emxArray_int32_T *idx,
  emxArray_real_T *x, int32_T offset, int32_T n, int32_T preSortLevel,
  emxArray_int32_T *iwork, emxArray_real_T *xwork)
{
  int32_T nPairs;
  int32_T bLen;
  int32_T tailOffset;
  int32_T nTail;
  emlrtStack st;
  st.prev = sp;
  st.tls = sp->tls;
  nPairs = n >> preSortLevel;
  bLen = 1 << preSortLevel;
  while (nPairs > 1) {
    if ((nPairs & 1) != 0) {
      nPairs--;
      tailOffset = bLen * nPairs;
      nTail = n - tailOffset;
      if (nTail > bLen) {
        st.site = &vc_emlrtRSI;
        merge(&st, idx, x, offset + tailOffset, bLen, nTail - bLen, iwork, xwork);
      }
    }

    tailOffset = bLen << 1;
    nPairs >>= 1;
    for (nTail = 1; nTail <= nPairs; nTail++) {
      st.site = &xc_emlrtRSI;
      merge(&st, idx, x, offset + (nTail - 1) * tailOffset, bLen, bLen, iwork,
            xwork);
    }

    bLen = tailOffset;
  }

  if (n > bLen) {
    st.site = &yc_emlrtRSI;
    merge(&st, idx, x, offset, bLen, n - bLen, iwork, xwork);
  }
}

void sortIdx(const emlrtStack *sp, emxArray_real_T *x, emxArray_int32_T *idx)
{
  int32_T ib;
  int32_T m;
  int32_T n;
  int32_T b_n;
  int32_T i;
  emxArray_int32_T *iwork;
  real_T x4[4];
  int32_T idx4[4];
  emxArray_real_T *xwork;
  int32_T nNaNs;
  boolean_T overflow;
  int32_T k;
  int32_T wOffset;
  int8_T perm[4];
  int32_T nNonNaN;
  int32_T i4;
  int32_T nBlocks;
  int32_T bLen2;
  int32_T nPairs;
  int32_T b_iwork[256];
  real_T b_xwork[256];
  int32_T exitg1;
  emlrtStack st;
  emlrtStack b_st;
  emlrtStack c_st;
  emlrtStack d_st;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  c_st.prev = &b_st;
  c_st.tls = b_st.tls;
  d_st.prev = &c_st;
  d_st.tls = c_st.tls;
  emlrtHeapReferenceStackEnterFcnR2012b(sp);
  ib = x->size[0];
  m = idx->size[0];
  idx->size[0] = ib;
  emxEnsureCapacity_int32_T(sp, idx, m, &w_emlrtRTEI);
  for (m = 0; m < ib; m++) {
    idx->data[m] = 0;
  }

  st.site = &jc_emlrtRSI;
  n = x->size[0];
  b_st.site = &kc_emlrtRSI;
  b_n = x->size[0];
  for (i = 0; i < 4; i++) {
    x4[i] = 0.0;
    idx4[i] = 0;
  }

  emxInit_int32_T(&b_st, &iwork, 1, &ab_emlrtRTEI, true);
  m = iwork->size[0];
  iwork->size[0] = ib;
  emxEnsureCapacity_int32_T(&b_st, iwork, m, &w_emlrtRTEI);
  for (m = 0; m < ib; m++) {
    iwork->data[m] = 0;
  }

  emxInit_real_T1(&b_st, &xwork, 1, &bb_emlrtRTEI, true);
  i = x->size[0];
  m = xwork->size[0];
  xwork->size[0] = i;
  emxEnsureCapacity_real_T1(&b_st, xwork, m, &w_emlrtRTEI);
  for (m = 0; m < i; m++) {
    xwork->data[m] = 0.0;
  }

  nNaNs = 0;
  ib = 0;
  c_st.site = &pc_emlrtRSI;
  overflow = ((!(1 > x->size[0])) && (x->size[0] > 2147483646));
  if (overflow) {
    d_st.site = &x_emlrtRSI;
    check_forloop_overflow_error(&d_st);
  }

  for (k = 0; k < b_n; k++) {
    if (muDoubleScalarIsNaN(x->data[k])) {
      idx->data[(b_n - nNaNs) - 1] = k + 1;
      xwork->data[(b_n - nNaNs) - 1] = x->data[k];
      nNaNs++;
    } else {
      ib++;
      idx4[ib - 1] = k + 1;
      x4[ib - 1] = x->data[k];
      if (ib == 4) {
        i = k - nNaNs;
        if (x4[0] <= x4[1]) {
          ib = 1;
          m = 2;
        } else {
          ib = 2;
          m = 1;
        }

        if (x4[2] <= x4[3]) {
          wOffset = 3;
          i4 = 4;
        } else {
          wOffset = 4;
          i4 = 3;
        }

        if (x4[ib - 1] <= x4[wOffset - 1]) {
          if (x4[m - 1] <= x4[wOffset - 1]) {
            perm[0] = (int8_T)ib;
            perm[1] = (int8_T)m;
            perm[2] = (int8_T)wOffset;
            perm[3] = (int8_T)i4;
          } else if (x4[m - 1] <= x4[i4 - 1]) {
            perm[0] = (int8_T)ib;
            perm[1] = (int8_T)wOffset;
            perm[2] = (int8_T)m;
            perm[3] = (int8_T)i4;
          } else {
            perm[0] = (int8_T)ib;
            perm[1] = (int8_T)wOffset;
            perm[2] = (int8_T)i4;
            perm[3] = (int8_T)m;
          }
        } else if (x4[ib - 1] <= x4[i4 - 1]) {
          if (x4[m - 1] <= x4[i4 - 1]) {
            perm[0] = (int8_T)wOffset;
            perm[1] = (int8_T)ib;
            perm[2] = (int8_T)m;
            perm[3] = (int8_T)i4;
          } else {
            perm[0] = (int8_T)wOffset;
            perm[1] = (int8_T)ib;
            perm[2] = (int8_T)i4;
            perm[3] = (int8_T)m;
          }
        } else {
          perm[0] = (int8_T)wOffset;
          perm[1] = (int8_T)i4;
          perm[2] = (int8_T)ib;
          perm[3] = (int8_T)m;
        }

        idx->data[i - 3] = idx4[perm[0] - 1];
        idx->data[i - 2] = idx4[perm[1] - 1];
        idx->data[i - 1] = idx4[perm[2] - 1];
        idx->data[i] = idx4[perm[3] - 1];
        x->data[i - 3] = x4[perm[0] - 1];
        x->data[i - 2] = x4[perm[1] - 1];
        x->data[i - 1] = x4[perm[2] - 1];
        x->data[i] = x4[perm[3] - 1];
        ib = 0;
      }
    }
  }

  wOffset = (b_n - nNaNs) - 1;
  if (ib > 0) {
    for (i = 0; i < 4; i++) {
      perm[i] = 0;
    }

    if (ib == 1) {
      perm[0] = 1;
    } else if (ib == 2) {
      if (x4[0] <= x4[1]) {
        perm[0] = 1;
        perm[1] = 2;
      } else {
        perm[0] = 2;
        perm[1] = 1;
      }
    } else if (x4[0] <= x4[1]) {
      if (x4[1] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 2;
        perm[2] = 3;
      } else if (x4[0] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 3;
        perm[2] = 2;
      } else {
        perm[0] = 3;
        perm[1] = 1;
        perm[2] = 2;
      }
    } else if (x4[0] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 1;
      perm[2] = 3;
    } else if (x4[1] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 3;
      perm[2] = 1;
    } else {
      perm[0] = 3;
      perm[1] = 2;
      perm[2] = 1;
    }

    c_st.site = &qc_emlrtRSI;
    if (ib > 2147483646) {
      d_st.site = &x_emlrtRSI;
      check_forloop_overflow_error(&d_st);
    }

    for (k = 1; k <= ib; k++) {
      idx->data[(wOffset - ib) + k] = idx4[perm[k - 1] - 1];
      x->data[(wOffset - ib) + k] = x4[perm[k - 1] - 1];
    }
  }

  m = (nNaNs >> 1) + 1;
  c_st.site = &rc_emlrtRSI;
  for (k = 1; k < m; k++) {
    i = idx->data[wOffset + k];
    idx->data[wOffset + k] = idx->data[b_n - k];
    idx->data[b_n - k] = i;
    x->data[wOffset + k] = xwork->data[b_n - k];
    x->data[b_n - k] = xwork->data[wOffset + k];
  }

  if ((nNaNs & 1) != 0) {
    x->data[wOffset + m] = xwork->data[wOffset + m];
  }

  nNonNaN = n - nNaNs;
  i = 2;
  if (nNonNaN > 1) {
    if (n >= 256) {
      nBlocks = nNonNaN >> 8;
      if (nBlocks > 0) {
        b_st.site = &lc_emlrtRSI;
        for (i4 = 1; i4 <= nBlocks; i4++) {
          b_st.site = &mc_emlrtRSI;
          nNaNs = (i4 - 1) << 8;
          for (b_n = 0; b_n < 6; b_n++) {
            n = 1 << (b_n + 2);
            bLen2 = n << 1;
            nPairs = 256 >> (b_n + 3);
            c_st.site = &sc_emlrtRSI;
            for (k = 1; k <= nPairs; k++) {
              ib = nNaNs + (k - 1) * bLen2;
              c_st.site = &tc_emlrtRSI;
              for (i = 0; i < bLen2; i++) {
                b_iwork[i] = idx->data[ib + i];
                b_xwork[i] = x->data[ib + i];
              }

              wOffset = 0;
              m = n;
              i = ib - 1;
              do {
                exitg1 = 0;
                i++;
                if (b_xwork[wOffset] <= b_xwork[m]) {
                  idx->data[i] = b_iwork[wOffset];
                  x->data[i] = b_xwork[wOffset];
                  if (wOffset + 1 < n) {
                    wOffset++;
                  } else {
                    exitg1 = 1;
                  }
                } else {
                  idx->data[i] = b_iwork[m];
                  x->data[i] = b_xwork[m];
                  if (m + 1 < bLen2) {
                    m++;
                  } else {
                    i = (i - wOffset) + 1;
                    c_st.site = &uc_emlrtRSI;
                    while (wOffset + 1 <= n) {
                      idx->data[i + wOffset] = b_iwork[wOffset];
                      x->data[i + wOffset] = b_xwork[wOffset];
                      wOffset++;
                    }

                    exitg1 = 1;
                  }
                }
              } while (exitg1 == 0);
            }
          }
        }

        i = nBlocks << 8;
        ib = nNonNaN - i;
        if (ib > 0) {
          b_st.site = &nc_emlrtRSI;
          merge_block(&b_st, idx, x, i, ib, 2, iwork, xwork);
        }

        i = 8;
      }
    }

    b_st.site = &oc_emlrtRSI;
    merge_block(&b_st, idx, x, 0, nNonNaN, i, iwork, xwork);
  }

  emxFree_real_T(&st, &xwork);
  emxFree_int32_T(&st, &iwork);
  emlrtHeapReferenceStackLeaveFcnR2012b(sp);
}

/* End of code generation (sortIdx.c) */
