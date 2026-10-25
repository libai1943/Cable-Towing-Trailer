# Zhang et al. RA-L 2026 strategy transfer: retain search mode sequence.
subject to fixed_slack{t in I:zseed[t]=0}: W[t]=0;
subject to fixed_slack_side{t in I:zseed[t]=0}: e[t]<=0;
subject to fixed_taut{t in I:zseed[t]=1}: W[t]=e[t];
