
def chaincode(a):
	chain=[]
	prev=a[0]
	next=a[1]
	#create chain code
	for i in range(1,len(a)-1):
		if prev[0]==next[0] and prev[1]>next[1]:
			chain.append('L')
		elif prev[0]==next[0] and prev[1]<next[1]:
			chain.append('R')
		elif prev[0]<next[0] and prev[1]==next[1]:
			chain.append('B')
		elif prev[0]>next[0] and prev[1]==next[1]:
			chain.append('F')
		prev=a[i]
		next=a[i+1]
		
	#compress the chain code to send to bot
	fchain=[]
	comp=8
	l,r,f,b=0,0,0,0
	for i in chain:
		if i=='F':
			f+=1
			if f==comp:
				f=0
				fchain.append('F')
		elif i=='L':
			l+=1
			if l==comp:
				l=0
				fchain.append('L')
		elif i=='R':
			r+=1
			if r==comp:
				r=0
				fchain.append('R')
		elif i=='B':
			b+=1
			if b==comp:
				b=0
				fchain.append('B')
	return fchain