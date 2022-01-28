
dict_sizes = {
	1: 6.7,
	#2: 6.7,
	2: 6.7,
	6: 6.7,
	7: 6.7,
	42: 10, #tag central
	52: 10, #tag Réservé à l'équipe violette
	53: 10, #tag Réservé à l'équipe violette
	42: 10, #tag central
	17: 5, #face cachée - all
	47: 5, #face tresor rouge
	13: 5, #face tresor bleu
	36: 5, #face tresor vert
	65: 5
 }


'''
FL	FR

RL	RR
'''
#FL FR RL RR
FL = 1
FR = 2
RL = 6
RR = 7
tagsOnRobot = [FL,FR,RL,RR]
pairsTag = [[FR,FL],[RR,FR],[RL,RR],[FL,RL]]
distanceBetweenPairTags = [16.2,14.7,16.8,14.2]
#entre FL et FR : 16.2cm
#entre FL et RL L14.3
#entre RL et RR : 16.8cm
#entre FR et RR : 14.7cm


#anglePairTag = [0,90,180,270]#avec 0 vers la droite
anglePairTag = [270,0,90,180] #avec 0 face camera





#BGR
dictColors ={
    1: (255,255,255),
	2: (255,255,255),
	6: (255,255,255),
	7: (255,255,255),
	52: (255,0,255), #tag Réservé à l'équipe violette
	53: (255,0,255), #tag Réservé à l'équipe violette
	42: (0,0,255), #tag central
	17: (0,255,255), #face cachée - all
	47: (0,0,255), #face tresor rouge
	13: (255,0,0), #face tresor bleu
	36: (0,255,0), #face tresor vert
	65: (255,255,255),
}