import qrcode

print('Packages imported prolerly!')

code = qrcode.make('ENPM 809T!')

code.save('enpm809Tqrcode.png')

print('QR Code generated and saved!')
