var Z{I} binary;
subject to lower_branch{t in I}: W[t]>=e[t];
subject to binary_slack{t in I}: W[t]<=smax*Z[t];
subject to binary_taut{t in I}: W[t]<=e[t]+L*(1-Z[t]);
