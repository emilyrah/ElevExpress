library(stringr)

runSigmaModel <- function(nfloors, nelev, elevcap, idleflor, delay, ntrials=100){

numbers <- str_pad(1:ntrials, 5, pad="0")
filename <- paste("myData", numbers, ".csv", sep="")
noyes <- rep("n", ntrials)
rs <- str_pad(sample(1:65535, ntrials, replace=FALSE),5, pad="0")
stoptime <- 720
traceYN <- 1
myData <- cbind(filename
			   , noyes
			   , rs
			   , rep(stoptime,ntrials)
			   , rep(traceYN,ntrials)
			   , rep(nfloors,ntrials)
			   , rep(nelev,ntrials)
			   , rep(elevcap,ntrials)
			   , rep(idleflor,ntrials)
			   , rep(delay,ntrials)
				)
				
myData <- as.vector(t(myData))


write(myData,file="ElevatorModelFinalRun.exp",ncolumns=10,append=FALSE,sep="\t")
x <- system("ElevatorModelFinalRun.exe")

runData <- list()
for(i in 1:length(filename)){

runData[[i]] <- read.csv(filename[i], sep='')

} 

return(runData)
}

runData <- runSigmaModel(10, 5, 10, 1, 0.5, 10)