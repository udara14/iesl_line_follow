//----Left Branch detection---
  if (s1 == 0 && s2 == 0) {
    Serial.print(F("[FOLLOW] left branch detected! branch #"));
    Serial.println(branchCount + 1);
    branchCount++;
    currentBranchDirection = -1; // left
    turnLeft = true; // set turn direction for this branch
    motorStop();
    delay(100);
    enterState(TURNING);
    return;
  }

  //----Right Branch detection---
  if (s4 == 0 && s5 == 0) {
    Serial.print(F("[FOLLOW] right branch detected! branch #"));
    Serial.println(branchCount + 1);
    branchCount++;
    currentBranchDirection = 1; // right  
    turnLeft = false; // set turn direction for this branch
    motorStop();
    delay(100);
    enterState(TURNING);
    return;
  }

  //--- T junction detection ---
if(allBlack && currentBranchDirection != 0){
    Serial.print(F("[FOLLOW] T-junction detected! This is branch #"));
    Serial.println(branchCount + 1);
    if (currentBranchDirection == -1) {
      turnLeft = true; // set turn direction for left branch
    } else if (currentBranchDirection == 1) {
      turnLeft = false; // set turn direction for right branch
    }
    motorStop();
    delay(100);
    enterState(TURNING);
    return;
  }