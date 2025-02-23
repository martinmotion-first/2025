function getBlueAllianceTeamData() {
  const allDataContainingRange = "A2:Q100";
  const stLouisEventKey = "2023mosl";
  //const stLouisEventKey = "2024mosl";
  const peoriaEventKey = "2024utwv";
  //const peoriaEventKey = "2024ilpe";

  var stLouisTeamsUrl = "https://www.thebluealliance.com/api/v3/event/" + stLouisEventKey + "/teams";  //st louis
  var stLouisSpreadsheetKey = "St Louis";
  var stLouisSheet = SpreadsheetApp.getActive().getSheetByName(stLouisSpreadsheetKey);
  var stLouisRankingsDataUrl = "https://www.thebluealliance.com/api/v3/event/" + stLouisEventKey + "/rankings";

  var peoriaTeamsUrl = "https://www.thebluealliance.com/api/v3/event/" + peoriaEventKey + "/teams"; //peoria
  var peoriaSpreadsheetKey = "Peoria";
  var peoriaSheet = SpreadsheetApp.getActive().getSheetByName(peoriaSpreadsheetKey);
  var peoriaRankingsDataUrl = "https://www.thebluealliance.com/api/v3/event/" + peoriaEventKey + "/rankings";

  var options = {
    'method' : 'get',
    'contentType': 'application/json',
    'headers': {
        'X-TBA-Auth-Key': 'zbrk7XkI2Ju60Xrz8KgzM3T8XNrZ6EiGs5eQ2VBZhm0LaAJ9kZIouEs5Kw9SQM7F'
      }
  };

  loadTeamDataForEvent(stLouisSpreadsheetKey, stLouisSheet, stLouisTeamsUrl, options);
  loadTeamDataForEvent(peoriaSpreadsheetKey, peoriaSheet, peoriaTeamsUrl, options);

  loadRankingsDataForEvent(stLouisSpreadsheetKey, stLouisSheet, stLouisRankingsDataUrl, options);
  loadRankingsDataForEvent(peoriaSpreadsheetKey, peoriaSheet, peoriaRankingsDataUrl, options);

  sortSheet(peoriaSheet, allDataContainingRange);
  sortSheet(stLouisSheet, allDataContainingRange);
}

function clearItAll(sheets, rangeDefinition){
  for(var i = 0; i < sheets.length; i++){
    sheets[i].getRange(rangeDefinition).clearContent();
  }
}

function sortSheet(spreadsheet, rangeDefinition){
  spreadsheet.getRange(rangeDefinition).sort([
    {column: 5, ascending: true},
    {column: 1, ascending: true}
  ]);
}

function loadRankingsDataForEvent(key, spreadsheet, url, options){
  var rankingsResponse = UrlFetchApp.fetch(url, options);
  if (rankingsResponse.getResponseCode() !== 200) {
    Logger.log("Failed to fetch rankings data for " + key + ". Response code: " + rankingsResponse.getResponseCode());
    return;
  }
  var rankingsParsed = JSON.parse(rankingsResponse.getContentText()).rankings;
  Logger.log(key + " rankings data:" + JSON.stringify(rankingsParsed));
  var lastRow = spreadsheet.getLastRow();
  var teamKeys = spreadsheet.getRange('D2:D' + lastRow).getValues();
  for(var i = 0; i < teamKeys.length; i++){
    var key = teamKeys[i];
    if(!key || !key.toString().trim()){
      continue;
    }
    var node = rankingsParsed.find(r => r.team_key == key);
    if(!node){
      continue;
    }
    Logger.log(key + " node.sort_orders: " + JSON.stringify(node.sort_orders));
    var cellKey = i + 2;
    spreadsheet.getRange("E" + cellKey).setValue(node.rank);
    spreadsheet.getRange("F" + cellKey).setValue(node.record.wins);
    spreadsheet.getRange("G" + cellKey).setValue(node.record.losses);
    spreadsheet.getRange("H" + cellKey).setValue(node.record.ties);
    spreadsheet.getRange("I" + cellKey).setValue(node.matches_played || ''); // Ensure to handle null or undefined
    spreadsheet.getRange("J" + cellKey).setValue(node.dq);
    
    // Check if node.sort_orders exists before accessing its properties
    if (node.sort_orders) {
      spreadsheet.getRange("K" + cellKey).setValue(node.sort_orders[0] || ''); //total
      spreadsheet.getRange("L" + cellKey).setValue(node.sort_orders [1] || ''); //ranking_score
      spreadsheet.getRange("M" + cellKey).setValue(node.sort_orders [2] || ''); //coopertition_points
      spreadsheet.getRange("N" + cellKey).setValue(node.sort_orders [3] || ''); //match_points
      spreadsheet.getRange("O" + cellKey).setValue(node.sort_orders [4] || ''); //auto_points
      spreadsheet.getRange("P" + cellKey).setValue(node.sort_orders [5] || ''); //teleop_points
      spreadsheet.getRange("Q" + cellKey).setValue(node.sort_orders [6] || ''); //endgame_points
    } else {
      Logger.log("Sort orders not found for " + key);
    }
  }
}

function loadTeamDataForEvent(key, spreadsheet, url, options){
  var responseData = UrlFetchApp.fetch(url, options);
  if (responseData.getResponseCode() !== 200) {
    Logger.log("Failed to fetch team data for " + key + ". Response code: " + responseData.getResponseCode());
    return;
  }
  var parsedData = JSON.parse(responseData.getContentText());
  var dataToLog = parsedData.map(function(team) {
    return JSON.stringify(team);
  }).join(", ");
  Logger.log(key + " team data:" + dataToLog);
  for(var i = 0; i < parsedData.length; i++){
    var rowNumber = i + 2;
    spreadsheet.getRange('A' + rowNumber).setValue(parsedData[i].nickname);
    spreadsheet.getRange('B' + rowNumber).setValue(parsedData[i].team_number);
    spreadsheet.getRange('C' + rowNumber).setValue(parsedData[i].rookie_year);
    spreadsheet.getRange('D' + rowNumber).setValue(parsedData[i].key);
  }
}


