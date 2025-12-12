import random

tutulan_sayi = random.randint(1, 100)

print("1 ile 100 arasında bir sayı tuttum. Hadi tahmin et!")

sayac = 0 

while True:
    try:

        tahmin = int(input("Tahminin nedir?: "))
        sayac += 1

        if tahmin < tutulan_sayi:
            print("Daha YUKARI çık!")
        
        elif tahmin > tutulan_sayi:
            print("Daha AŞAĞI in!")
            
        else:
            print(f" Tebrikler! Sayıyı {sayac}. denemede buldun.")
            break 

    except ValueError:
        print("Lütfen geçerli bir sayı giriniz!")