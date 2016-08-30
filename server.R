library(shiny)
source("helperShiny.r")
shinyServer(function(input, output) {
  mydb <- reactive({
    nflor <- input$nflor;
    minelev <- input$slider2[1];
    maxelev <- input$slider2[2];
    elevcap <- input$elevcap;
    idle <- input$idle;
    delay <-input$num;
    

    if (minelev == maxelev){    
     runSigmaModel(nflor, minelev, elevcap, idle, delay, 10)
    } else {
     for (i in input$slider2[1]:input$slider2[2]) {
       runSigmaModel(nflor, i, elevcap, idle, delay, 10)
      }
    }
  })
  ## We couldn't figure out how to have the interface communicate with our output in time for the project.
  output$Plot1 <- renderPlot({});
  output$Plot2 <- renderPlot({});
  output$Plot3 <- renderPlot({});

})