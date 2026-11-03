from pathlib import Path
import struct
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).parent
source=Path('/Users/gbq38u/Desktop/spider_gaming/детальки/main_frame.STL')
b=source.read_bytes();n=struct.unpack_from('<I',b,80)[0]
font='/System/Library/Fonts/Supplemental/Arial Unicode.ttf'
f1=ImageFont.truetype(font,34);f2=ImageFont.truetype(font,25);f3=ImageFont.truetype(font,21)
W,H=1580,1450;s=4.55;ox=62;oy=68+275.84*s
im=Image.new('RGB',(W,H),'white');d=ImageDraw.Draw(im)
for i in range(n):
 v=struct.unpack_from('<12fH',b,84+50*i)
 pts=[(round(ox+v[j]*s),round(oy-v[j+2]*s)) for j in (3,6,9)]
 d.polygon(pts,fill=(231,234,237),outline=(196,202,207))
for tx,ax,col in [(29.543,85.0,(31,103,161)),(156.043,101.0,(212,94,47))]:
 x0,y0=ox+tx*s,oy-249.112*s
 x1,y1=ox+ax*s,oy-254.5*s
 d.line((x0,y0,x1,y1),fill=col,width=7)
 d.ellipse((x0-17,y0-17,x0+17,y0+17),fill='white',outline=col,width=5)
# front bar datum and four holes for each bracket
for group,xc,col in [('ЛЕВАЯ',73.0,(31,103,161)),('ПРАВАЯ',112.5,(212,94,47))]:
 for x in ((81,89) if group=='ЛЕВАЯ' else (97,105)):
  for z in (251,258):
   px=ox+x*s;py=oy-z*s;r=7.0
   d.ellipse((px-r,py-r,px+r,py+r),fill=col,outline='white',width=2)
 d.text((1010,280 if group=='ЛЕВАЯ' else 410),group,font=f1,fill=col)
 for ii,x in enumerate((81,89) if group=='ЛЕВАЯ' else (97,105)):
  y0=335 if group=='ЛЕВАЯ' else 465
  d.text((1010,y0+ii*31),f'X = {x:.1f}; Z = 251 / 258',font=f2,fill=(40,48,56))
# dashed front reference line at z=254
py=oy-254*s
for px in range(55,900,15):d.line((px,py,min(px+8,900),py),fill=(76,92,103),width=2)
d.text((920,py-14),'ось поперечины Z = 254',font=f3,fill=(76,92,103))
d.text((50,15),'РАЗМЕТКА ОТВЕРСТИЙ M3 НА ИСХОДНОЙ РАМЕ',font=f1,fill=(28,39,49))
d.line((980,85,980,1360),fill=(214,220,225),width=3)
d.text((1010,105),'Координаты в main_frame.STL, мм',font=f2,fill=(38,48,58))
d.text((1010,180),'X — поперёк; Z — вперёд',font=f2,fill=(38,48,58))
d.text((1010,600),'Сверло: Ø 3.3–3.4 мм',font=f2,fill=(38,48,58))
d.text((1010,650),'Все точки на сплошной',font=f2,fill=(38,48,58))
d.text((1010,685),'поперечине рамы:',font=f2,fill=(38,48,58))
d.text((1010,735),'толщина ≈ 20.5 мм',font=f2,fill=(38,48,58))
d.text((1010,805),'Перемычки идут на 30 мм',font=f2,fill=(38,48,58))
d.text((1010,840),'выше поверхности рамы',font=f2,fill=(38,48,58))
d.text((1010,930),'Всего 8 отверстий:',font=f2,fill=(38,48,58))
d.text((1010,965),'по 4 на каждую клешню',font=f2,fill=(38,48,58))
d.text((1010,1065),'Примерить на реальной раме',font=f2,fill=(170,67,37))
d.text((1010,1100),'до сверления.',font=f2,fill=(170,67,37))
d.text((130,1330),'ПЕРЕД  +Z  ↑',font=f1,fill=(40,74,116))
d.text((1010,1320),'План, не для печати в масштабе 1:1',font=f3,fill=(95,103,110))
im.save(ROOT/'preview'/'mount_drilling.png')
