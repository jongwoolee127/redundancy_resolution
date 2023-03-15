def joint_imp(Kq: float, q0: float, q: float, Bq: float, dq0: float, dq: float ):
    return Kq @ ( q0 - q ) + Bq @ ( dq0 - dq )