import crtk, dvrk
import numpy as np

ral = crtk.ral('dvrk_python_node')

ral.check_connections()
ral.spin()

psm2 = dvrk.psm(ral, 'PSM2')
ecm = dvrk.ecm(ral, 'ECM')
mtml = dvrk.mtm(ral, 'MTML')

psm2.enable(10) and ecm.enable(10) and mtml.enable(10)
psm2.home(10) and ecm.home(10) and mtml.home(10)

ecm.move_jp(np.deg2rad([0, 0, np.rad2deg(0.156), 0]))
psm2.move_jp(np.deg2rad([-70, -5, np.rad2deg(0.147), 90, 15, 0]))

ral.shutdown()