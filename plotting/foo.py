from codec import *
from hamming import *

MSG = b"Somebody once told me the world is gonna roll me / I ain't the sharpest tool in the shed / She was looking kind of dumb with her finger and her thumb / In the shape of an \"L\" on her forehead / Well, the years start comin' and they don't stop comin' / Fed to the rules and I hit the ground runnin' / Didn't make sense not to live for fun / Your brain gets smart but your head gets dumb / So much to do, so much to see / So what's wrong with taking the backstreets? / You'll never know if you don't go / You'll never shine if you don't glow / Hey now, you're an all star / Get your game on, go play / Hey now, you're a rock star / Get your show on, get paid / (And all that glitters is gold) / Only shootin' stars break the mold / It's a cool place, and they say it gets colder / You're bundled up now, wait 'til you get older / But the meteor men beg to differ / Judging by the hole in the satellite picture / The ice we skate is gettin' pretty thin / The waters gettin' warm so you might as well swim / My world's on fire, how 'bout yours? / That's the way I like it and I'll never get bored / Hey now, you're an all star / Get your game on, go play / Hey now, you're a rock star / Get your show on, get paid / (All that glitters is gold) / Only shootin' stars break the mold / Go for the moon / G-g-g-go for the moon / Go for the moon / Go-go-go for the moon / Hey now, you're an all star / Get your game on, go play / Hey now, you're a rock star / Get your show on, get paid / (All that glitters is gold)."


exp = BitString.from_bytes(len(MSG).to_bytes(2, 'little') + MSG)
res = BitString(0, 0)

while exp.bits > 0:
    res.push_msb(BitString(hamming_encode_7_4(exp.pop_lsb(4, strict=False)), 7))

b = res.to_bytes(strict=False)

print(','.join([f'0x{a:02X}' for a in b]))