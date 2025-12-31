from typing import Final

from zigpy import types as t
from zigpy.quirks.v2 import QuirkBuilder
from zigpy.quirks.v2.homeassistant import EntityType, UnitOfTime
from zigpy.zcl.foundation import BaseAttributeDefs, ZCLAttributeDef
from zigpy.zcl import foundation

from zhaquirks import CustomCluster

ESP_MANUF = "prax19"
ESP_MODEL = "Retrofit GC"
EP = 10

class GateConfigCluster(CustomCluster):
    """Manufacturer-specific config cluster exposed by the device."""
    cluster_id: Final[t.uint16_t] = 0xFF00
    
    manufacturer_id_override: t.uint16_t = foundation.ZCLHeader.NO_MANUFACTURER_ID

    class AttributeDefs(BaseAttributeDefs):
        # gate_transition_ds = ZCLAttributeDef(
        #     id=0x0000,
        #     type=t.uint16_t,
        #     access="rwp",  # read/write + reportable
        #     # is_manufacturer_specific=True,
        # )
        beam_debounce_ms = ZCLAttributeDef(
            id=0x0001,
            type=t.uint16_t,
            access="rwp",
            # is_manufacturer_specific=True,
        )

(
    QuirkBuilder(ESP_MANUF, ESP_MODEL)
    .replace_cluster_occurrences(GateConfigCluster)

    # .number(
    #     GateConfigCluster.AttributeDefs.gate_transition_ds.name,
    #     GateConfigCluster.cluster_id,
    #     endpoint_id=EP,
    #     translation_key="gate_transition_time",
    #     min_value=0.5,
    #     max_value=30.0,
    #     step=0.5,
    #     unit=UnitOfTime.SECONDS,
    #     mode="box",
    #     multiplier=0.1,
    #     entity_type=EntityType.CONFIG,
    #     fallback_name="Czas przejścia bramy",
    # )

    .number(
        GateConfigCluster.AttributeDefs.beam_debounce_ms.name,
        GateConfigCluster.cluster_id,
        endpoint_id=EP,
        translation_key="beam_debounce",
        min_value=0,
        max_value=2000,
        step=10,
        mode="box",
        entity_type=EntityType.CONFIG,
        fallback_name="Debounce fotokomórki (ms)",
    )

    .add_to_registry()
)