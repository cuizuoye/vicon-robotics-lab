import threading
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
from model import Limits
from ros_tuning import install,instrument


class DashboardTests(unittest.TestCase):
    def test_ready_requires_supervisor_same_epoch_report(self):
        log=NS(emit=lambda *a,**k:None)
        module=install(log,Limits(),True)
        c=object.__new__(module.Chain)
        c.sup=NS(core=NS(epoch='new',gate_report=dict(epoch='old',fault='',hard_fault=False)))
        with patch.object(instrument.BaseChain,'ready',return_value=True):
            self.assertFalse(c.ready())
            c.sup.core.gate_report['epoch']='new'
            self.assertTrue(c.ready())
            c.sup.core.gate_report['fault']='fault'
            self.assertFalse(c.ready())

    def test_persistent_drawing_and_no_automatic_start(self):
        import tkinter as tk
        from dashboard import Window
        actions=[];records=[]
        worker=NS(snapshot={},ui_stamp=0.,profile=Limits(.7,4.,400.),done=threading.Event(),
            post=lambda action:actions.append(action),trace=NS(emit=lambda *args,**kw:records.append((args,kw))))
        root=tk.Tk();root.withdraw();window=Window(root,worker)
        try:
            root.update_idletasks()
            pose=dict(x=0.,y=0.,yaw=0.)
            s=dict(car=pose,goal=dict(x=.2,y=.1,yaw=.1),trace=dict(snapshot_seq=1,
                model_seq=1,oid=['gate',1],cid=['controller',1,'epoch']),error_m=.2,
                reference_limited=False,accepted_signed=1,profile=worker.profile.name)
            for i in range(200):
                s['car']=dict(x=i*.001,y=.05,yaw=.01)
                window.draw(s)
            self.assertLessEqual(len(window.canvas.find_all()),7)
            self.assertEqual(len(records),200)
            self.assertEqual(actions,[])
            self.assertLessEqual(root.winfo_reqheight(),720)
        finally:window.close()


if __name__=='__main__':unittest.main(verbosity=2)
