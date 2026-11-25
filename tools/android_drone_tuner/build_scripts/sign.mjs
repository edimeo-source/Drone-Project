// Sign with APK Signature Scheme v1 + v2 + v3 (SHA-256), key made with openssl.
import { readFileSync, writeFileSync } from 'node:fs';
const lib = await import('/home/claude/apkbuild/signer/node_modules/apk_sign_ts/dist/index.js');
const [inApk, outApk, keyPem, certPem] = process.argv.slice(2);
const signer = new lib.ApkSigner({
  signingKey: lib.SigningKey.fromPEM(readFileSync(keyPem, 'utf8'), readFileSync(certPem, 'utf8')),
  digestAlgorithm: 'SHA-256',
  jarKeyName: 'CERT',
});
const { signedApk } = await signer.sign(new Uint8Array(readFileSync(inApk)));
writeFileSync(outApk, signedApk);
console.log('signed', outApk, signedApk.length, 'bytes');
