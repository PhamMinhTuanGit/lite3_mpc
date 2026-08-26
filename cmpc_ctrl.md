### Mối quan hệ giữa các đại lượng điều khiển
- `freq`, `iterations_between_mpc`, `horizonLength`, `dt`
- dt = 1 \ freq
- iterations_between_mpc: liên đới tới tần số điều khiển của MPC
    - ví dụ: iterations_between_mpc = 30 -> freq/30 tần số điều khiển của MPC
- horizonLength: liên đới tới tầm nhìn của MPC và tổng chu kì của môt gait.
