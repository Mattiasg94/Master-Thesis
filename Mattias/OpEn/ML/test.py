from PIL import Image

background = Image.open("background.png")
foreground = Image.open("car2.png")

background.paste(foreground, (0, 0), foreground)
background.show()