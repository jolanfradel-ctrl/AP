#ifndef IFIRMWARE_SOURCE_H
#define IFIRMWARE_SOURCE_H

#include <Arduino.h>

// Interface : n'importe quelle source de firmware (WiFi, modem AT, SD...)
// doit savoir répondre à ces 4 questions, peu importe comment en interne.
class IFirmwareSource {
public:
    virtual ~IFirmwareSource() {}

    // Ouvre la connexion / prépare le téléchargement. Retourne false si échec.
    virtual bool begin(const String& url) = 0;

    // Taille totale annoncée du fichier (pour Update.begin()). -1 si inconnue.
    virtual int totalSize() = 0;

    // Lit jusqu'à 'len' octets dans 'buffer'. Retourne le nombre d'octets lus,
    // 0 si plus rien pour l'instant (mais pas fini), -1 si terminé/erreur.
    virtual int readChunk(uint8_t* buffer, size_t len) = 0;

    // Libère la connexion proprement.
    virtual void end() = 0;
};

#endif