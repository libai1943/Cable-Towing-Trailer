# Zeng et al. RA-L 2020 strategy transfer: nonnegative product complementarity.
subject to lower_branch{t in I}: W[t]>=e[t];
subject to complementarity{t in I}: W[t]*(W[t]-e[t])<=epscomp;
