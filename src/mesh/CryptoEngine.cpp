#include "CryptoEngine.h"
#include "configuration.h"

//concurrency::Lock *cryptLock;
#ifndef SILENT
    #define MSG(...) Serial.printf(__VA_ARGS__)
#else
    #define MSG(...)
#endif

void CryptoEngine::setKey(const CryptoKey &k)
{
    MSG("Using AES%d key!\n", k.length * 8);
    key = k;
}

/**
 * Encrypt a packet
 *
 * @param bytes is updated in place
 */
void CryptoEngine::encrypt(uint32_t fromNode, uint64_t packetId, size_t numBytes, uint8_t *bytes)
{
    MSG("noop encryption!\n");
}

void CryptoEngine::decrypt(uint32_t fromNode, uint64_t packetId, size_t numBytes, uint8_t *bytes)
{
    MSG("noop decryption!\n");
}

/**
 * Init our 128 bit nonce for a new packet
 */
void CryptoEngine::initNonce(uint32_t fromNode, uint64_t packetId)
{
    memset(nonce, 0, sizeof(nonce));

    // use memcpy to avoid breaking strict-aliasing
    memcpy(nonce, &packetId, sizeof(uint64_t));
    memcpy(nonce + sizeof(uint64_t), &fromNode, sizeof(uint32_t));
}