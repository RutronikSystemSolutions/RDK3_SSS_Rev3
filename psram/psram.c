/*
 * psram.c
 *
 *  Created on: 2024-03-21
 *      Author: Gintaras
 */

#include "psram.h"

#define SMIF_PRIORITY           (1u)     		/* SMIF interrupt priority */
#define TIMEOUT_1_MS            (1000ul) 		/* 1 ms timeout for all blocking functions */

void Isr_SMIF(void);
void Init_SMIF(void);

/* SMIF configuration parameters */
cy_stc_smif_config_t SMIFConfig =
{
    /* .mode           */ CY_SMIF_NORMAL,      /* Mode of operation */
    /* .deselectDelay  */ 2U,      			/* Minimum duration of SPI deselection */
    /* .rxClockSel     */ CY_SMIF_SEL_INVERTED_INTERNAL_CLK,     /* Clock source for the receiver clock */
    /* .blockEvent     */ CY_SMIF_BUS_ERROR    /* What happens when there is a read
                                                * to an empty RX FIFO or write to a full TX FIFO
                                                */
};
cy_stc_smif_context_t smifContext;

/**/
void psram_init(void)
{

    /*Initializes SMIF block*/
    Init_SMIF();

    /*Enter XIP mode*/
    Cy_SMIF_SetMode(QSPI_MEM_HW, CY_SMIF_MEMORY);
}

/*******************************************************************************
* Function Name: Isr_SMIF
********************************************************************************
*
* The ISR for the SMIF interrupt. All Read/Write transfers to/from the external
* memory are processed inside the SMIF ISR.
*
*******************************************************************************/
void Isr_SMIF(void)
{
    Cy_SMIF_Interrupt(QSPI_MEM_HW, &smifContext);
}

/*******************************************************************************
* Function Name: Initialize_SMIF
********************************************************************************
*
* This function initializes the SMIF block
*
*******************************************************************************/
void Init_SMIF(void)
{
	/* Initialize SMIF interrupt */

	      cy_stc_sysint_t smifIntConfig =
		  {
		   .intrSrc = 2u,
		   .intrPriority = SMIF_PRIORITY
		  };

	      cy_en_sysint_status_t intrStatus = Cy_SysInt_Init(&smifIntConfig, Isr_SMIF);

	      if(0u != intrStatus)
		    {

		        for(;;)
		          {
		             /*Waits forever when SMIF initialization error occurs*/
		          }
		    }

		  /* Initialize SMIF */
		  cy_en_smif_status_t smifStatus;
		  smifStatus = Cy_SMIF_Init(QSPI_MEM_HW, &SMIFConfig, TIMEOUT_1_MS, &smifContext);

		  if(0u != smifStatus)
		    {

		   for(;;)
		         {
		             /*Waits forever when SMIF initialization error occurs*/
		         }
		     }
	     /* Configure slave select and data select. These settings depend on the pins
	      * selected in the Device and QSPI configurators.
	      */
		   Cy_SMIF_SetDataSelect(QSPI_MEM_HW, APS1604M_3SQR_ZR_SlaveSlot_0.slaveSelect, APS1604M_3SQR_ZR_SlaveSlot_0.dataSelect);
	       Cy_SMIF_Enable(QSPI_MEM_HW, &smifContext);
	       Cy_SMIF_Memslot_Init(QSPI_MEM_HW, (cy_stc_smif_block_config_t *)&smifBlockConfig, &smifContext);
	       Cy_SMIF_SetMode(QSPI_MEM_HW, CY_SMIF_NORMAL);
	       NVIC_EnableIRQ(smifIntConfig.intrSrc); /* Enable the SMIF interrupt */
}


