#coding:utf-8

""" """
from tkinter import *
from tkinter import messagebox
from tkinter import filedialog
import time
# Definition des paramètres de notre fénetre
mainApp = Tk()
mainApp.title("Carte du marraudeur")
mainApp.geometry("640x400")
mainApp.resizable(width=False, height=False)
flag = 0
# Fonction relative aux options de notre fenetre
def function():
	donnee.delete(ALL)
	Texte = donnee.create_text(350, 50, text="En Attente de données...", font="Arial 16 italic", fill="blue")
	start.configure(state='disabled')
def Start():
	Texte = donnee.create_text(370, 50, text="Démarrage de l'application...", font="Arial 16 italic", fill="blue")
	donnee.after(3000, function)
	
	#time.sleep(1)
	#Texte = donnee.create_text(370, 50, text="En Attente de données...", font="Arial 16 italic", fill="blue")
def Save():
	fichier = filedialog.asksaveasfile(title="Sauvegarder le fichier", filetypes=[('Fichier txt','.txt')])
	print(fichier.name)
def Open():
	fichier = filedialog.askopenfilename(title="Ouvrir un fichier", filetypes=[('Fichier txt','.txt')])
	print(fichier)
def A_propos():
	a_propos = messagebox.showinfo("La Carte du Marraudeur","Ce Programme a pour but de suivre des utilisateurs dans le batiment de l'université et de pouvoir tracer leur trajetoire :)")
	
def Quit():
	quitter = messagebox.askokcancel("Attention","Voulez-vous quitter l'application ?") 
	if quitter == True:
		mainApp.quit()
#Création de nos différents widgets
cardreMenu = Menu(mainApp)

file = Menu(cardreMenu, tearoff=0)
cardreMenu.add_cascade(label="File", menu=file)
file.add_command(label="Save", command=Save)
file.add_separator()
file.add_command(label="Open", command=Open)

Apropos = Menu(cardreMenu, tearoff=0)
cardreMenu.add_cascade(label="About", menu=Apropos)
Apropos.add_command(label="About Sofware", command = A_propos)

donnee = Canvas(mainApp,bg='dark grey',height=370, width=635)
image = PhotoImage(file = 'la_carte_du_maraudeur.gif')
donnee.create_image(70,80, anchor=NW, image=image)
donnee.pack()
start = Button(mainApp,text='Start',width =8, command=Start)
start.pack(side=LEFT, padx=5)
quit = Button(mainApp,text='Quit', width =8, command=Quit)
quit.pack(side=LEFT)


mainApp.config(menu=cardreMenu)

mainApp.mainloop()