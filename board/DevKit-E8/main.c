/*---------------------------------------------------------------------------
 * Copyright (c) 2025-2026 Arm Limited (or its affiliates). All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * DevKit-E8 (M55_HP) board bring-up for a headless Ethos-U85 runner. Derived
 * from the pack's Boards/DevKit-e8/Layers/M55_HP/main.c without the MIPI,
 * USB, Ethernet and VIO initialisation.
 *---------------------------------------------------------------------------*/

#include "RTE_Components.h"
#include CMSIS_device_header

#include "board_config.h"
#include "main.h"

#include "se_services_port.h"
#include "services_lib_api.h"
#include <stdio.h>

/* SRAM0 is off at reset when booting from MRAM without a debugger. The NPU can only
 * reach SRAM (not the core's DTCM), so the ExecuTorch pools live there: power it
 * on before anything uses it. */
static void sram0_power_on(void)
{
    uint32_t err = 0;
    uint32_t ret = SERVICES_power_memory_req(se_services_s_handle, POWER_MEM_SRAM_0_ENABLE, &err);

    printf("SRAM0 power on: ret %lu, err %lu\n", (unsigned long)ret, (unsigned long)err);
}

int main(void)
{
    /* Apply the Conductor pin configuration (includes the UART4 console pins) */
    board_pins_config();

    /* Apply the Conductor GPIO configuration */
    board_gpios_config();

    /* Bring up the Secure Enclave services (MHU link to the SE) */
    se_services_port_init();

    /* Request the clocks the SE has to enable for this core */
    board_clocks_config(CLKEN_HFOSC_MASK | CLKEN_CLK_100M_MASK);

    /* Initialize STDIO (UART4 on the PRG USB connector) */
    stdio_init();
    sram0_power_on();

    #if defined(ETHOSU_ARCH)
    /* Initialize Ethos NPU */
    ethos_setup();
    #endif

    return app_main();
}
