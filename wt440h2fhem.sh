#!/bin/bash

HOST=localhost
PORT=7072

while read housecode channel status battery humid temp; do
  batt_txt="ok"
  batt_stat=""
  hum_prefix=""
  if [ $battery -eq 1 ]; then
    batt_txt="low"
    batt_stat=", Bat.low!"
  fi
  if [ $humid -le 14 ]; then
    hum_prefix="<="
  fi
  echo -ne "setreading wt440h_${housecode}_${channel} temperature ${temp}\nsetreading wt440h_${housecode}_${channel} humidity ${humid}\nsetreading wt440h_${housecode}_${channel} battery ${batt_txt}\nset wt440h_${housecode}_${channel} ${temp}°C, ${hum_prefix}${humid}%${batt_stat}\n" | nc ${HOST} ${PORT}
done
