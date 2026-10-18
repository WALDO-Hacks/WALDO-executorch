/* The YOLOv8n ExecuTorch program (model/yolo/yolo.pte, 3 MB), embedded as is: a C array
   of it would be an 18 MB source file. Read-only data in MRAM, 64-byte aligned for the
   NPU. YOLOv8 is AGPL-3.0 (Ultralytics). */
    .section .rodata.yolo_pte, "a"
    .balign 64
    .global yolo_pte
yolo_pte:
    .incbin YOLO_PTE_PATH   /* model/yolo/yolo.pte, set in cmsis-executorch.cproject.yml */
yolo_pte_end:
    .balign 4
    .global yolo_pte_size
yolo_pte_size:
    .word yolo_pte_end - yolo_pte
