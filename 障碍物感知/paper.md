# V. Simulation Analysis

This section addresses three questions: whether the stage-wise rollout degrades performance in a frozen-favorable case, whether it improves pure tracking when horizon-varying non-inertial effects become stronger, and whether the same benefit carries over to the CBF-constrained setting.

### A. Simulation Setup and Evaluation Protocol

We compare the proposed `SNI-MPC-CBF` controller with the frozen non-inertial baseline `NI-Frozen MPC-CBF`, and we also include the no-CBF ablations `NI-Stage MPC` and `NI-Frozen MPC` to isolate the effect of the stage-wise rollout. All controllers share the same plant model, prediction horizon, sampling period, solver settings, and active UAV set (`active_uavs=1,2,3`). All runs include state-estimation noise and first-order actuator lag.

The simulation study uses three scenarios: a circular baseline, an obstacle-free figure-eight, and a fixed-obstacle figure-eight. Figure 1 visualizes these layouts.

The operating conditions are parameterized by `KP`, with `KP=1,2,3,4,6` corresponding to `V=W=0.5,1.0,1.5,2.0,3.0`, respectively. We group these operating points as low speed (`KP=1`), medium speed (`KP=2,3`), and high speed (`KP=4,6`).

![Fig. 1](figures/paper_fig1_scenarios.png)

**Fig. 1.** Simulation scenarios used in the paired-seed study. Panels (a), (b), and (c) show the circular baseline, the obstacle-free figure-eight, and the fixed-obstacle figure-eight, respectively. The screenshots illustrate the UGV path, the UAV formation, and the obstacle placement used in the safety-constrained case.

For the safety-constrained task, a trial is counted as a failure when the minimum planar surface distance satisfies `d < r_uav + r_obs + 0.215 m`, equivalently when the physical surface clearance falls below `0.215 m`. Binary safety is reported as `failure count / 100` and the corresponding rate.

### B. Comparative Studies

Table I provides the primary quantitative comparison, whereas Fig. 2 is used as a qualitative contrastive illustration of the relative-frame tracking-error trajectories.

![Fig. 2](figures/paper_fig1_relative_grid.png)

**Fig. 2.** Representative relative-frame tracking-error trajectories from the paired-seed simulations. The black marker denotes the desired relative reference point, the blue curve denotes the frozen counterpart, and the red curve denotes the stage-wise counterpart. Panels (a)-(e), (f)-(j), and (k)-(o) correspond to the circular baseline, the obstacle-free figure-eight, and the fixed-obstacle figure-eight with soft CBF, respectively. Within each row, the panels follow `KP=1,2,3,4,6`, grouped as low, medium, and high speed. All panels in the same row share a common square axis range. Local zoomed insets are included where needed for clarity. For each operating point, the displayed seed maximizes the team-level frozen-minus-stage tracking-RMS difference, and the displayed UAV maximizes the corresponding per-UAV difference within that seed. Accordingly, Fig. 2 should be interpreted as a contrastive case-study figure rather than a median-seed summary of the full distribution.

**B1. Circular baseline.** On the circular baseline, which is the scenario most favorable to the frozen rollout, the low- and medium-speed groups show virtually no separation between the two no-CBF controllers. At low speed, the mean tracking RMS is `0.04531 ± 0.00026 m` for the frozen controller and `0.04532 ± 0.00026 m` for the stage-wise controller at `KP=1`. This near-overlap persists through the medium-speed group and remains small even at the milder high-speed point, where the RMS values are `0.12619 ± 0.00145 m` versus `0.12765 ± 0.00144 m` at `KP=4`. Only at the most extreme high-speed case (`KP=6`) do both no-CBF controllers exhibit a sharp RMS increase, reaching `5.49538 ± 0.01309 m` for frozen and `5.63415 ± 0.01322 m` for stage. The main conclusion is therefore that the stage-wise rollout does not introduce noticeable degradation on the circular baseline until both controllers enter the extreme high-speed regime.

**B2. Obstacle-free figure-eight.** The obstacle-free figure-eight is the clearest advantage case for the stage-wise rollout, and the benefit strengthens monotonically from the low-speed group to the medium-speed group and then to the high-speed group. At low speed, the improvement is positive but small (`+1.884%` at `KP=1`). Across the medium-speed group, the improvement becomes clearly visible, reaching `+12.262%` and `+20.438%` at `KP=2` and `KP=3`. Across the high-speed group, it becomes large, reaching `+22.954%` at `KP=4` and `+50.068%` at `KP=6` (Table I). Representative absolute tracking-RMS reductions are from `0.04599 ± 0.00015 m` to `0.04512 ± 0.00014 m` at `KP=1`, from `0.08355 ± 0.00059 m` to `0.06647 ± 0.00054 m` at `KP=3`, and from `0.49335 ± 0.00547 m` to `0.24634 ± 0.00191 m` at `KP=6`. This speed-group trend is consistent with the intended mechanism: as the non-inertial terms vary more strongly over the prediction horizon, the stage-wise rollout becomes increasingly beneficial.

**B3. Fixed-obstacle figure-eight with soft CBF.** Under active CBF, the same trend carries into the constrained setting, but the gain is concentrated much more strongly in the high-speed group. At low speed, the tracking-RMS improvement is positive but marginal (`+0.100%` at `KP=1`). Across the medium-speed group, the improvement remains modest, at `+0.712%` and `+0.928%` for `KP=2` and `KP=3`. Across the high-speed group, however, the improvement becomes substantially larger, reaching `+7.999%` at `KP=4` and `+32.116%` at `KP=6` (Table I). The constrained metrics follow the same pattern: the mean tracking RMS decreases from `0.37280 ± 0.02989 m` to `0.34298 ± 0.02999 m` at `KP=4` and from `1.53005 ± 0.36730 m` to `1.03865 ± 0.24235 m` at `KP=6`, while the mean slack sum decreases from `621.95` to `385.01` at `KP=4` and from `943.23` to `377.35` at `KP=6`. The safety-violation count is also lower for the stage-wise controller throughout the non-saturated range, improving from `14/100` to `13/100` at `KP=1`, from `20/100` to `17/100` at `KP=2`, from `63/100` to `52/100` at `KP=3`, and from `99/100` to `86/100` at `KP=4`. At `KP=6`, the binary metric saturates at `100/100` for both controllers; in that regime, the mean minimum surface distance (`0.088 ± 0.026 m` versus `0.102 ± 0.028 m`) and the slack sum become the more informative safety indicators. Taken together, these results indicate that the rollout advantage observed in the obstacle-free figure-eight also carries into the CBF-constrained setting.

**Table I. Summary of Main Simulation Results**

| Scene | Metric | `KP=1` | `KP=2` | `KP=3` | `KP=4` | `KP=6` |
| --- | --- | --- | --- | --- | --- | --- |
| Circle, no CBF | Tracking-RMS improvement | `-0.010%` | `-0.194%` | `-0.674%` | `-1.153%` | `-2.525%` |
| Figure-eight, no CBF | Tracking-RMS improvement | `+1.884%` | `+12.262%` | `+20.438%` | `+22.954%` | `+50.068%` |
| Figure-eight, with CBF | Tracking-RMS improvement | `+0.100%` | `+0.712%` | `+0.928%` | `+7.999%` | `+32.116%` |
| Figure-eight, with CBF | Slack-sum reduction | `+0.636%` | `+0.383%` | `+0.112%` | `+38.096%` | `+59.994%` |
| Figure-eight, with CBF | Safety-violation rate (<0.215 m) | `0.140000 / 0.130000` | `0.200000 / 0.170000` | `0.630000 / 0.520000` | `0.990000 / 0.860000` | `1.000000 / 1.000000` |

Tracking RMS and slack sum are reported as relative improvement only; the corresponding absolute means and across-seed dispersions are given in the discussion above.
Safety-violation rate is reported as `Frozen / Stage` and uses a `0.215 m` minimum-surface-distance threshold; at `KP=6` the binary metric saturates at `1.0 / 1.0`.

### C. Simulation-Side Computational Overhead

The simulation-side timing remains comparable between the two non-inertial controllers. Across all reported groups, the stage-wise controller yields mean solve times of `1.91-4.30 ms` and mean `P95` solve times of `4.40-7.78 ms`, compared with `1.98-4.27 ms` and `4.38-7.80 ms` for the frozen controller. The worst per-UAV `P95` remains below `10.14 ms` for the stage-wise controller and below `11.59 ms` for the frozen controller.

Using the logged `>20 ms` threshold, the mean over-`20 ms` rate remains below `0.035%` for the stage-wise controller and below `0.193%` for the frozen controller over the whole study. Therefore, the stage-wise rollout does not introduce a systematic computational penalty in simulation, and in the higher-speed constrained cases it is slightly faster. This timing scale is also consistent with the `50 Hz` hardware execution reported in the full manuscript.

\section{Hardware Experiments and Validation}
\label{sec:hardware}

\subsection{Experimental Setup and Evaluation Scope}
\label{sec:hardware_setup}

Hardware validation is conducted on an indoor motion-capture platform with one UGV and three UAVs, as shown in Figs.~\ref{fig:hardware_platform} and~\ref{fig:hardware_scene}. We compare the proposed `SNI-MPC-CBF` controller with the frozen non-inertial baseline `NI-Frozen MPC-CBF` under obstacle-free and fixed-obstacle lemniscate runs. The UGV executes a replayed lemniscate command trace from the same trajectory family used in simulation, and both controllers run online at $50$~Hz with sampling interval $\Delta t=0.02$~s. The three UAVs track fixed target-relative offsets $[0.0,\ 0.5,\ 1.0]^\top$, $[0.0,\ -0.5,\ 1.0]^\top$, and $[0.5,\ 0.0,\ 1.0]^\top$ in the UGV-attached frame. For each controller, the hardware evaluation is based on five obstacle-free laps and five fixed-obstacle laps recorded over repeated lemniscate runs. We report team-level tracking RMS, worst-lap max-UAV RMS, minimum obstacle and inter-UAV surface distances, and online MPC solve time.

\begin{figure}[!b]
    \centering
    \includegraphics[width=\columnwidth,keepaspectratio]{figures/fig4.png}
    \caption{Hardware platforms used in the experiments: the UGV and the quadrotor.}
    \label{fig:hardware_platform}
\end{figure}

\begin{figure}[!b]
    \centering
    \includegraphics[width=\columnwidth,keepaspectratio]{figures/fig6.pdf}
    \caption{Representative first-lap hardware time-series for the obstacle-free and fixed-obstacle lemniscate runs. Top: target-relative position and tracking error. Bottom: fixed-obstacle traces with nearest-obstacle surface distance.}
    \label{fig:hardware_tracking_examples}
\end{figure}

\subsection{Obstacle-Free lemniscate Tracking Results}
\label{sec:hardware_obstacle_free}

Across five obstacle-free laps, `SNI-MPC-CBF` achieved a team target-relative tracking RMS of $0.295 \pm 0.010~\mathrm{m}$, compared with $0.331 \pm 0.022~\mathrm{m}$ for `NI-Frozen MPC-CBF`, corresponding to a $10.85\%$ reduction. The worst-lap max-UAV RMS was $0.346~\mathrm{m}$ for `SNI-MPC-CBF` and $0.456~\mathrm{m}$ for `NI-Frozen MPC-CBF`. The logged MPC solve time remained similar, with P95 values of $1.29 \pm 0.05~\mathrm{ms}$ and $1.31 \pm 0.15~\mathrm{ms}$, respectively. The top row of Fig.~\ref{fig:hardware_tracking_examples} shows representative first-lap target-relative position and axis-wise error traces, confirming stable three-UAV formation maintenance before obstacle constraints are introduced.

\subsection{Fixed-Obstacle lemniscate Results}
\label{sec:hardware_fixed_obstacle}

Across five fixed-obstacle laps, `SNI-MPC-CBF` achieved a team tracking RMS of $0.367 \pm 0.018~\mathrm{m}$, compared with $0.419 \pm 0.023~\mathrm{m}$ for `NI-Frozen MPC-CBF`, corresponding to a $12.6\%$ reduction. The worst-lap max-UAV RMS was $0.496~\mathrm{m}$ for `SNI-MPC-CBF` and $0.584~\mathrm{m}$ for `NI-Frozen MPC-CBF`. The minimum obstacle and inter-UAV surface distances were $0.156~\mathrm{m}$ and $0.157~\mathrm{m}$ for `SNI-MPC-CBF`, compared with $0.127~\mathrm{m}$ and $0.144~\mathrm{m}$ for `NI-Frozen MPC-CBF`. The logged MPC solve time remained comparable, with P95 values of $4.48~\mathrm{ms}$ and $4.55~\mathrm{ms}$, and peak values of $9.62~\mathrm{ms}$ and $10.46~\mathrm{ms}$, respectively. The bottom row of Fig.~\ref{fig:hardware_tracking_examples} shows representative first-lap target-relative position traces together with the nearest-obstacle surface-distance evolution from one fixed-obstacle run.
