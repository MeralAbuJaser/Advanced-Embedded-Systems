# invoke SourceDir generated makefile for TIRTOS_demo.pem4f
TIRTOS_demo.pem4f: .libraries,TIRTOS_demo.pem4f
.libraries,TIRTOS_demo.pem4f: package/cfg/TIRTOS_demo_pem4f.xdl
	$(MAKE) -f C:\Users\Meral\workspace_v10\empty_min_EK_TM4C123GXL_TI/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\Meral\workspace_v10\empty_min_EK_TM4C123GXL_TI/src/makefile.libs clean

