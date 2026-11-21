# Working robot checkpoint — 2026-11-18

Firmware: `direct-rollback-ir-v1`. User reports very good operation after correcting the centre IR sensor, which had read floor over tape during earlier failed tests. This is user-reported physical validation, not an autonomous course test.

The saved source matches the last uploaded driving logic and GUI. No new firmware was uploaded to make this checkpoint. The unuploaded straight-balance experiment is saved locally in `.git/unuploaded-straight-balance.patch` and is not part of this version.

## Last captured runtime tuning

Captured and restored after the IR GUI upload, before the user's latest test:

- `LINESPEED 142 132 95 79` (straight, lost-line outside, correction outside, gentle inside; mm/s)
- `LINECFG 98 0` (reference PWM, minimum PWM)
- Compiled motor profile: reference speed 200, offset 18.82, P 0.2353, I 0.5647, launch PWM 61. Serial reports rounded P/I as 0.24/0.56.

The user may have adjusted sliders since this capture; these values are not asserted to be the exact successful-run settings. Tuning is RAM-only. A reboot uses source defaults (250/250/250/125 and reference PWM 66), not the captured runtime values. Reapply the two commands above while stopped to reproduce the last captured values.

## Validation limits

The direct follower retains its simple last-side recovery. This checkpoint does not establish completion of every hairpin or course stress simulation. Prior failed experiments remain historical evidence, not proof of a software cause for the startup failure with a misadjusted centre sensor.
