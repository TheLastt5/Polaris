sayi1 = float(input("Lütfen ilk sayıyı girin: "))

sayi2 = float(input("Lütfen ikinci sayıyı girin: "))

toplam = sayi1 + sayi2

fark = sayi1 - sayi2

carpim = sayi1 * sayi2

if sayi2 != 0:
    bolum = sayi1 / sayi2
else:
    bolum = "Tanımsız (Sıfıra Bölme)"

print("\n--- Hesaplama Sonuçları ---")
print(f"Sayı 1: {sayi1}")
print(f"Sayı 2: {sayi2}")
print("-" * 30)

print(f"Toplama ({sayi1} + {sayi2}): {toplam}")
print(f"Fark ({sayi1} - {sayi2}): {fark}")
print(f"Çarpım ({sayi1} * {sayi2}): {carpim}")
print(f"Bölüm ({sayi1} / {sayi2}): {bolum}")