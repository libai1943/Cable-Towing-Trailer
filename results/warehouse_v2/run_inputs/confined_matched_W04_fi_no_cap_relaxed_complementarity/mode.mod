# Formulation control: the exact nonnegative product graph of the common max law.
subject to lower_branch{t in I}: W[t]>=e[t];
subject to complementarity{t in I}: W[t]*(W[t]-e[t])<=epscomp;
