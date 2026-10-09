"""ace2k_tags: the brand modules, the registry, the key derivations, the [ace2k_tag] sections —
with synthetic UIDs and synthetic dumps only (no real tag's UID ever enters the tree)."""

import configparser
import struct
from pathlib import Path

import ace2k_tag
import pytest
from ace2k_tags import base, crypto, registry

CFG = Path(__file__).resolve().parents[3] / "config" / "ace2k_tags.cfg"
UID4 = bytes.fromhex("01020304")
UID7 = bytes.fromhex("04112233445566")
NTAG = base.Chip(uid=UID7, atqa=0x0044, sak=0x00)
M1K = base.Chip(uid=UID4, atqa=0x0004, sak=0x08)


def sections():
    """The shipped file's sections as {brand: {option: value}}, as ace2k_tag.py hands them."""
    cp = configparser.ConfigParser(interpolation=None)
    cp.read(CFG)
    out = {}
    for name in cp.sections():
        kind, brand = name.split()
        assert kind == "ace2k_tag"
        out[brand] = dict(cp[name])
    return out


def brand(name):
    return registry.BRANDS[name](sections()[name])


def test_aes_matches_fips_197_and_inverts():
    key = bytes(range(16))
    pt = bytes.fromhex("00112233445566778899aabbccddeeff")
    ct = crypto.aes128_encrypt_block(key, pt)
    assert ct.hex() == "69c4e0d86a7b0430d8cdb78070b4c55a"
    assert crypto.aes128_decrypt_block(key, ct) == pt


def test_hkdf_matches_rfc_5869_case_1():
    ikm = bytes([0x0B] * 22)
    salt = bytes(range(13))
    info = bytes(range(0xF0, 0xFA))
    okm = crypto.hkdf_sha256(ikm, salt, info, 42)
    assert okm.hex() == (
        "3cb25f25faacd57a90434f64d0362f2a2d2d0a90cf1a5a4c5db02d56ecc4c5bf34007208d5b887185865"
    )


def test_every_brand_parameter_lives_in_the_cfg_with_a_source():
    s = sections()
    assert set(s) == {"anycubic", "bambu", "creality", "elegoo", "snapmaker"}
    for name, opts in s.items():
        assert opts.get("source", "").startswith("https://"), name
    for py in (Path(crypto.__file__).parent).glob("*.py"):
        text = py.read_text()
        for secret in ("9a759cf2", "qwertyuiop", "713362755e", "484043466b"):
            assert secret not in text, (py.name, secret)


def test_key_derivations_for_a_synthetic_uid():
    bambu = brand("bambu")
    assert bambu.key_for(UID4, 0).hex() == "6b0d673986de"
    assert bambu.key_for(UID4, 1).hex() == "9c41cdfa0a4d"
    assert bambu.key_for(UID4, 15).hex() == "be9da70ee88e"
    snap = brand("snapmaker")
    assert snap.key_for(UID4, 0).hex() == "f658de759175"
    assert snap.key_for(UID4, 1).hex() == "561b70517aaf"
    assert brand("creality").key_for(UID4, 1).hex() == "440cef12b194"


def test_claims_by_chip_type():
    for name in ("anycubic", "elegoo"):
        assert brand(name).claims(NTAG) and not brand(name).claims(M1K)
    for name in ("bambu", "snapmaker", "creality"):
        assert brand(name).claims(M1K) and not brand(name).claims(NTAG)


def ntag_got(pages):
    """A Got holding NTAG pages {page: 4 bytes}."""
    got = base.Got()
    for p, b in pages.items():
        got.pages[p] = bytes(b)
    return got


def ntag_image(img, first=4):
    """Pages from a byte image starting at page `first`."""
    img = img + bytes((-len(img)) % 4)
    return {first + i // 4: img[i : i + 4] for i in range(0, len(img), 4)}


def anycubic_image():
    img = bytearray(144)
    img[0:4] = b"\x7b\x00\x65\x00"
    img[4:14] = b"AHPLBK-101"
    img[24:26] = b"AC"
    img[44:47] = b"PLA"
    struct.pack_into("<I", img, 64, (0x12 << 24) | (0x34 << 16) | (0x56 << 8))
    struct.pack_into("<HH", img, 80, 190, 230)
    struct.pack_into("<HH", img, 100, 55, 65)
    struct.pack_into("<H", img, 104, 175)
    struct.pack_into("<I", img, 108, 1000)
    return bytes(img)


def test_anycubic_reads_the_user_area_in_three_steps_and_decodes():
    a = brand("anycubic")
    got = base.Got()
    steps = a.steps(NTAG, got)
    assert [(s.op, s.arg, s.blocks) for s in steps] == [
        (base.OP_NTAG, 4, 3),
        (base.OP_NTAG, 16, 3),
        (base.OP_NTAG, 28, 3),
    ]
    got = ntag_got(ntag_image(anycubic_image()))
    for p in range(4, 40):
        got.pages.setdefault(p, bytes(4))
    assert a.steps(NTAG, got) == []
    rec = a.decode(NTAG, got)
    assert rec.format == "anycubic" and rec.uid == UID7.hex().upper()
    assert rec.brand == "AC" and rec.material == "PLA" and rec.sku == "AHPLBK-101"
    assert rec.color_rgba == "123456FF"
    assert (rec.hotend_min_c, rec.hotend_max_c, rec.bed_min_c, rec.bed_max_c) == (190, 230, 55, 65)
    assert rec.diameter_mm == 1.75 and rec.weight_g == 1000
    other = ntag_got(ntag_image(bytes(144)))
    assert a.decode(NTAG, other) is None


def elegoo_image():
    """OpenRFID's aligned layout at page 16: the marker, the material in ASCII, the colour, the
    hot-end range, the diameter and the weight, big-endian."""
    blk = bytearray(48)
    blk[1:5] = b"\xee\xee\xee\xee"
    blk[8:12] = b"PETG"
    blk[16:20] = bytes.fromhex("FF8800FF")
    struct.pack_into(">HH", blk, 20, 230, 260)
    struct.pack_into(">HH", blk, 28, 175, 1000)
    img = bytearray(24 * 4)
    img[48:96] = blk
    return bytes(img)


def test_elegoo_decodes_its_marker_and_declines_without_it():
    e = brand("elegoo")
    got = ntag_got(ntag_image(elegoo_image()))
    for p in range(4, 40):
        got.pages.setdefault(p, bytes(4))
    rec = e.decode(NTAG, got)
    assert rec.brand == "Elegoo" and rec.material == "PETG"
    assert rec.color_rgba == "FF8800FF"
    assert (rec.hotend_min_c, rec.hotend_max_c) == (230, 260)
    assert rec.diameter_mm == 1.75 and rec.weight_g == 1000
    assert e.decode(NTAG, ntag_got(ntag_image(anycubic_image()))) is None


def mifare_got(blocks):
    got = base.Got()
    for b, v in blocks.items():
        got.blocks[b] = bytes(v)
    return got


def bambu_blocks():
    b = {i: bytearray(16) for i in (1, 2, 4, 5, 6, 12, 14)}
    b[1][0:6] = b"A00-K0"
    b[1][8:13] = b"GFA00"
    b[2][0:3] = b"PLA"
    b[4][0:9] = b"PLA Basic"
    b[5][0:4] = bytes.fromhex("000000FF")
    struct.pack_into("<H", b[5], 4, 1000)
    struct.pack_into("<f", b[5], 8, 1.75)
    struct.pack_into("<HHHHHH", b[6], 0, 55, 8, 1, 60, 240, 190)
    b[12][0:16] = b"2025_10_17_14_26"
    struct.pack_into("<H", b[14], 4, 330)
    return b


def test_bambu_asks_sector_by_sector_with_its_keys_and_decodes():
    bb = brand("bambu")
    got = base.Got()
    (s0,) = bb.steps(M1K, got)
    assert (s0.op, s0.arg, s0.blocks, s0.key.hex()) == (base.OP_MIFARE_A, 0, 0b110, "6b0d673986de")
    blocks = bambu_blocks()
    got = mifare_got({1: blocks[1], 2: blocks[2]})
    (s1,) = bb.steps(M1K, got)
    assert (s1.arg, s1.blocks) == (1, 0b111)
    got = mifare_got(blocks)
    assert bb.steps(M1K, got) == []
    rec = bb.decode(M1K, got)
    assert rec.brand == "Bambu Lab" and rec.material == "PLA" and rec.name == "PLA Basic"
    assert rec.color_rgba == "000000FF" and rec.weight_g == 1000 and rec.diameter_mm == 1.75
    assert (rec.dry_temp_c, rec.dry_hours) == (55, 8)
    assert (rec.hotend_min_c, rec.hotend_max_c) == (190, 240)
    assert rec.length_m == 330 and rec.sku == "GFA00"
    assert rec.extra["production"] == "2025_10_17_14_26"


def test_a_failed_authentication_makes_a_brand_decline():
    bb = brand("bambu")
    got = base.Got()
    got.auth_failed.add((0, base.KEY_A))
    assert bb.steps(M1K, got) == []
    assert bb.decode(M1K, got) is None


def snapmaker_blocks():
    img = bytearray(176)
    img[16:25] = b"Snapmaker"
    img[32:41] = b"Polymaker"
    struct.pack_into("<HHH", img, 64, 1, 1, 3)
    img[72] = 1
    img[80:83] = bytes.fromhex("F4C032")
    struct.pack_into("<I", img, 96, 900003)
    struct.pack_into("<HHH", img, 128, 175, 500, 150)
    struct.pack_into("<HHHH", img, 144, 55, 6, 230, 190)
    return {b: img[b * 16 : b * 16 + 16] for b in range(11) if b % 4 != 3}


def test_snapmaker_reads_three_sectors_and_decodes_its_record():
    sm = brand("snapmaker")
    got = mifare_got(snapmaker_blocks())
    assert sm.steps(M1K, got) == []
    rec = sm.decode(M1K, got)
    assert rec.brand == "Snapmaker" and rec.material == "PLA" and rec.name == "SnapSpeed"
    assert rec.color_rgba == "F4C032FF"
    assert rec.diameter_mm == 1.75 and rec.weight_g == 500 and rec.length_m == 150
    assert (rec.dry_temp_c, rec.dry_hours) == (55, 6)
    assert (rec.hotend_min_c, rec.hotend_max_c) == (190, 230)
    assert rec.sku == "900003" and rec.extra["manufacturer"] == "Polymaker"
    assert rec.extra["signature"] == "unchecked"


def test_creality_decrypts_sector_one_and_decodes_the_ascii_record():
    cr = brand("creality")
    (s1,) = cr.steps(M1K, base.Got())
    assert (s1.arg, s1.blocks, s1.key.hex()) == (1, 0b111, "440cef12b194")
    ct = bytes.fromhex(
        "57f25b78076d4c1797b1be35ca269540c2381a51f9728d1e66baa1ff4781d677"
        "fac8f07509292df943d4cdf64cba06a1"
    )  # "AB1240276A210100100000FF016500000100000000000000", AES-128-ECB (openssl) under data_key
    got = mifare_got({4: ct[0:16], 5: ct[16:32], 6: ct[32:48]})
    rec = cr.decode(M1K, got)
    assert rec.brand == "Creality" and rec.sku == "101001"
    # date AB124, vendor 0276, batch A2, filament id 101001, colour 00000FF, length 0165,
    # serial 000001, reserve 000000, padding 00000000
    assert rec.color_rgba == "0000FFFF" and rec.extra["serial"] == "000001"
    assert rec.extra["vendor_id"] == "0276" and rec.extra["batch"] == "A2"
    assert rec.length_m == 165
    garbage = mifare_got({4: bytes(16), 5: bytes(16), 6: bytes(16)})
    assert cr.decode(M1K, garbage) is None


def test_the_shipped_cfg_enables_exactly_the_three_brands_checked_on_real_tags():
    s = sections()
    on = {name for name, opts in s.items() if registry._enabled(opts)}
    assert on == {"anycubic", "bambu", "snapmaker"}
    assert [f.name for f in registry.build(s)] == ["anycubic", "bambu", "snapmaker"]


def all_enabled():
    """The shipped sections with every brand switched on, Elegoo and Creality included."""
    s = sections()
    for opts in s.values():
        opts["enabled"] = "True"
    return s


def test_the_registry_orders_ntag_brands_then_the_mifare_order_and_skips_disabled():
    s = all_enabled()
    fmts = registry.build(s, ["creality", "bambu", "snapmaker"])
    assert [f.name for f in fmts] == ["anycubic", "elegoo", "creality", "bambu", "snapmaker"]
    s["bambu"]["enabled"] = "False"
    assert "bambu" not in [f.name for f in registry.build(s, ["bambu", "snapmaker", "creality"])]
    del s["snapmaker"]["salt_a"]
    assert "snapmaker" not in [f.name for f in registry.build(s, ["snapmaker", "creality"])]
    with pytest.raises(ValueError, match="nosuch"):
        registry.build(sections(), ["nosuch"])


def test_a_new_brand_is_one_module_and_one_register_call():
    class Demo(base.TagFormat):
        name = "demo"
        chip = "ntag"
        needs = ()

        def steps(self, chip, got):
            return base.ntag_steps(got, 4, 7)

        def decode(self, chip, got):
            if got.pages.get(4, b"")[:2] != b"DM":
                return None
            return base.SpoolRecord(format="demo", uid=chip.uid.hex().upper(), brand="Demo")

    registry.register(Demo)
    try:
        s = dict(sections(), demo={"source": "https://example.invalid/demo"})
        fmts = registry.build(s, ["bambu"])
        assert "demo" in [f.name for f in fmts]
        demo = [f for f in fmts if f.name == "demo"][0]
        got = ntag_got({4: b"DM\x00\x00", 5: bytes(4), 6: bytes(4), 7: bytes(4)})
        assert demo.decode(NTAG, got).brand == "Demo"
    finally:
        registry.BRANDS.pop("demo")


def test_the_section_module_reads_every_option_and_marks_them_read():
    class Section:
        def __init__(self, name, opts):
            self.name, self.opts, self.read = name, opts, set()

        def get_name(self):
            return self.name

        def get_prefix_options(self, prefix):
            return [k for k in self.opts if k.startswith(prefix)]

        def get(self, key, default=None):
            self.read.add(key)
            return self.opts.get(key, default)

        def getboolean(self, key, default=None):
            self.read.add(key)
            v = self.opts.get(key)
            return default if v is None else v.lower() in ("1", "true", "yes")

    sec = Section("ace2k_tag bambu", {"enabled": "True", "source": "https://x", "hkdf_salt": "aa"})
    t = ace2k_tag.load_config_prefix(sec)
    assert t.brand == "bambu" and t.enabled
    assert t.params == {"hkdf_salt": "aa", "source": "https://x"}
    assert sec.read == {"enabled", "source", "hkdf_salt"}
