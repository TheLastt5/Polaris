import pytest

from guvenlik_sistemi import sifre_kontrol

def test_guclu_sifre_onayi():
    ornek_sifre = "Guclu.Sifre123"
    assert sifre_kontrol(ornek_sifre) == True

def test_ozel_karakter_eksik():
    zayif_sifre = "SadeceHarfVeRakam1"
    assert sifre_kontrol(zayif_sifre) == False

def test_buyuk_harf_eksik():
    kucuk_sifre = "sifre123!"
    assert sifre_kontrol(kucuk_sifre) == False

def test_bos_sifre():
    assert sifre_kontrol("") == False