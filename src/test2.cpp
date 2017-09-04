#include <iostream>
#include <string>
using namespace std;

#include "crypto++/cryptlib.h"
#include "crypto++/filters.h"
#include "crypto++/files.h"
#include "crypto++/modes.h"
#include "crypto++/hex.h"
#include "crypto++/aes.h"
using namespace CryptoPP;

int main(int argc, char* argv[])
{
  byte key[AES::MAX_KEYLENGTH];
  byte iv[AES::BLOCKSIZE];
  vector<byte> plain, cipher, recover;
  HexEncoder encoder(new FileSink(cout));

  memset(key, 0x00, sizeof(key));
  memset(iv, 0x00, sizeof(iv));

  string str("Attack at dawn!");
  std::copy(str.begin(), str.end(), std::back_inserter(plain));

  cout << "Plain text: ";
  encoder.Put(plain.data(), plain.size());
  encoder.MessageEnd();
  cout << endl;

  /////////////////////////////////////////////////////////////

  CBC_Mode<AES>::Encryption enc;
  enc.SetKeyWithIV(key, sizeof(key), iv, sizeof(iv));

  // Make room for padding
  cipher.resize(plain.size()+AES::BLOCKSIZE);
  ArraySink cs(&cipher[0], cipher.size());

  ArraySource(plain.data(), plain.size(), true,
    new StreamTransformationFilter(enc, new Redirector(cs)));

  // Set cipher text length now that its known
  cipher.resize(cs.TotalPutLength());

  cout << "Cipher text: ";
  encoder.Put(cipher.data(), cipher.size());
  encoder.MessageEnd();
  cout << endl;

  /////////////////////////////////////////////////////////////

  CBC_Mode<AES>::Decryption dec;
  dec.SetKeyWithIV(key, sizeof(key), iv, sizeof(iv));

  // Recovered text will be less than cipher text
  recover.resize(cipher.size());
  ArraySink rs(&recover[0], recover.size());

  ArraySource(cipher.data(), cipher.size(), true,
    new StreamTransformationFilter(dec, new Redirector(rs)));

  // Set recovered text length now that its known
  recover.resize(rs.TotalPutLength());

  cout << "Recovered text: ";
  encoder.Put(recover.data(), recover.size());
  encoder.MessageEnd();
  cout << endl;

  return 0;
}
