# Optional one-factor softplus replacement only. No epsilon tolerance inflation.
subject to smooth_max{t in I}: W[t]=log(1+exp(smoothbeta*e[t]))/smoothbeta;
