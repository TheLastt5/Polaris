sayi = float(input("Lütfen bir sayı girin: "))

if sayi > 0:
    # Sayı sıfırdan büyükse (pozitif)
    print("Bu sayı POZİTİF bir sayıdır. (+)")

elif sayi < 0:
    # Sayı sıfırdan küçükse (negatif)
    print("Bu sayı NEGATİF bir sayıdır. (-)")

else:
    # Eğer ne pozitif ne de negatifse, sıfırdır.
    print("Bu sayı SIFIR'dır. (0)")