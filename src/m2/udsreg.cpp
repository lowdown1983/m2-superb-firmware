// M2 multi-mode firmware — UDS DID/RID registry data (Phase 9).
//
// Generic SAE/ISO DIDs compiled in as the default. DID names and security
// requirements per the OEM vehicle database (m2-oem-database v1.4.0,
// generated/json/dids.json, 'iso' entries); ISO 14229-1 for the DID
// number ranges (public standard). Make-specific packs come from
// scripts/oem_extract.py -> oem_gen.h and are consulted after this table.
//
// INFERRED on hardware (no bench).

#include "udsreg.h"
// Phase 9: make-specific DID pack from the OEM extraction pipeline
// (scripts/oem_extract.py -> oem_gen.h). Empty by default (generic build).
#include "oem_gen.h"

namespace udsreg {

namespace {

const DidEntry DIDS[] = {
    // ISO 14229-1 standardized 0xF18x-0xF19x range (public standard).
    { 0xF180, "BootSoftwareIdentification", 0 },
    { 0xF181, "ApplicationSoftwareIdentification", 0 },
    { 0xF182, "ApplicationDataIdentification", 0 },
    { 0xF183, "BootSoftwareFingerprint", DID_SEC },
    { 0xF184, "ApplicationSoftwareFingerprint", DID_SEC },
    { 0xF185, "ApplicationDataFingerprint", DID_SEC },
    { 0xF186, "ActiveDiagnosticSession", 0 },
    { 0xF187, "ManufacturerSparePartNumber", DID_ASCII },
    { 0xF188, "ManufacturerECUSoftwareNumber", DID_ASCII },
    { 0xF189, "ManufacturerECUSoftwareVersion", DID_ASCII },
    { 0xF18A, "SystemSupplierIdentifier", DID_ASCII },
    { 0xF18B, "ECUManufacturingDate", DID_ASCII },
    { 0xF18C, "ECUSerialNumber", DID_ASCII },
    { 0xF18D, "SupportedFunctionalUnits", 0 },
    { 0xF18E, "ManufacturerKitAssemblyPartNumber", DID_ASCII },
    { 0xF190, "VehicleIdentificationNumber", DID_ASCII },
    { 0xF191, "VehicleManufacturerECUHardwareNumber", DID_ASCII },
    { 0xF192, "SystemSupplierECUHardwareNumber", DID_ASCII },
    { 0xF193, "SystemSupplierECUHardwareVersion", DID_ASCII },
    { 0xF194, "SystemSupplierECUSoftwareNumber", DID_ASCII },
    { 0xF195, "SystemSupplierECUSoftwareVersion", DID_ASCII },
    { 0xF196, "ExhaustRegulationOrTypeApprovalNumber", DID_ASCII },
    { 0xF197, "SystemNameOrEngineType", DID_ASCII },
    { 0xF198, "RepairShopCodeOrTesterSerialNumber", DID_ASCII },
    { 0xF199, "ProgrammingDate", DID_ASCII },
    { 0xF19A, "CalibrationRepairShopCode", DID_ASCII },
    { 0xF19B, "CalibrationEquipmentSoftwareNumber", DID_ASCII },
    { 0xF19C, "ECUInstallationDate", DID_ASCII },
    { 0xF19D, "ODXFileIdentifier", DID_ASCII },
    { 0xF19E, "EntityIdentifier", DID_ASCII },
    // Commonly-implemented extras (SAE J1979 / industry practice).
    { 0xF40D, "VehicleSpeedSensorCalibration", 0 },
    { 0xF400, "ReadMemoryByAddressSetup", DID_SEC },
    { 0xF402, "ProgrammingStateIndicator", 0 },
};

const RidEntry RIDS[] = {
    // ISO 14229-1 routine examples (public standard); ECUs define their own.
    { 0xFF00, "EraseMemory" },
    { 0xFF01, "CheckProgrammingDependencies" },
    { 0xFF02, "EraseMirrorMemoryDTCs" },
    { 0x0202, "CheckSoftwareIntegrity" },
    { 0xE200, "ManufacturerSpecificRoutine" },
};

const uint16_t DIDS_N = sizeof(DIDS) / sizeof(DIDS[0]);
const uint16_t RIDS_N = sizeof(RIDS) / sizeof(RIDS[0]);

} // namespace (anonymous)

// Make-pack DID, adapted to the udsreg::DidEntry view. The oem table is
// consulted only when the built-in generic table misses.
static DidEntry makeView;
static bool makeViewValid = false;

const DidEntry* findDid(uint16_t did)
{
    for (uint16_t i = 0; i < DIDS_N; i++)
        if (DIDS[i].did == did) return &DIDS[i];
    for (uint16_t i = 0; i < oem::MAKE_DIDS_N; i++) {
        if (oem::MAKE_DIDS[i].did == did) {
            makeView.did = did;
            makeView.name = oem::MAKE_DIDS[i].name;
            makeView.flags = oem::MAKE_DIDS[i].flags;
            makeViewValid = true;
            return &makeView;
        }
    }
    makeViewValid = false;
    return nullptr;
}

const RidEntry* findRid(uint16_t rid)
{
    for (uint16_t i = 0; i < RIDS_N; i++)
        if (RIDS[i].rid == rid) return &RIDS[i];
    return nullptr;
}

bool decodeDid(uint16_t did, const uint8_t* d, uint8_t n,
               char* out, size_t cap)
{
    const DidEntry* e = findDid(did);
    if (e == nullptr || out == nullptr || cap == 0) return false;
    static const char* H = "0123456789ABCDEF";
    size_t p = 0;
    if (e->flags & DID_ASCII) {
        for (uint8_t i = 0; i < n && p + 1 < cap; i++) {
            char c = (char)d[i];
            out[p++] = (c >= 32 && c < 127) ? c : '.';
        }
    } else {
        for (uint8_t i = 0; i < n && p + 3 < cap; i++) {
            if (i) out[p++] = ' ';
            out[p++] = H[(d[i] >> 4) & 0xF];
            out[p++] = H[d[i] & 0xF];
        }
    }
    out[p] = '\0';
    return true;
}

uint16_t didCount() { return DIDS_N; }
const DidEntry* didAt(uint16_t i) { return (i < DIDS_N) ? &DIDS[i] : nullptr; }
uint16_t ridCount() { return RIDS_N; }
const RidEntry* ridAt(uint16_t i) { return (i < RIDS_N) ? &RIDS[i] : nullptr; }

} // namespace udsreg
