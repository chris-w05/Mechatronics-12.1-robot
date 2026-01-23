//Strategy.hpp
//This allows the robot to know it's current state in the game
//It keeps track of information such as what what level pickaxe/ sword is owned
#ifndef STRATEGY_H
#define STRATEGY_H

/*Board indexes
            0   1   2
chest side  3   4   5   start side
            6   7   8
            mine side
*/

/* Whenever Strategy is called, use it in this style:

Strategy::getInstance().setPickaxeLevel(Strategy::STONE);
auto maxBlock = Strategy::getInstance().getMaxMineableBlock();

I went with a singleton style over a static class since it keeps the nature of Strategy as an object, but means I don't need to pass it between all my classes
*/

/*Overall strategy:
    until _pickaxeLevel = DIAMOND:
        place two wood blocks on cells 1 and 4
        acquire 3 of highest available material, reject if not highest available block 
        if silverfish detected, kill
        place in cells 6, 7, 8
    
    else:
        get 1 wood
        get 2 of same block, reject if not same as first
        place in cells 6, 7, 8

*/


/*How block allocation should be done
    flag: blockRecieved
    When a block's type is measured, whereToIndexBlock should be called once.
    Then, when the block is mined, the placement of the block in the bucket can be allocated using the information from whereToIndexBlock
    W
*/



class Strategy{

    public :
        enum CycleType
        {
            PICKAXE_HANDLE,
            PICKAXE_TIP,
            SWORD,
            SHIELD
        };

        enum ToolLevel
        {
            WOOD_TOOL = 0,
            STONE_TOOL = 1,
            IRON_TOOL = 2,
            DIAMOND_TOOL = 3
        };

        enum Block
        {
            NONE = -1,
            WOOD = 0,
            STONE = 1,
            IRON = 2,
            DIAMOND = 3
        };

        enum IndexDirection
        {
            LEFT = 0,
            REJECT = -1,
            RIGHT = 2,
            HOLD = 1,
            CHEST = 3 // send rejected blocks to chest
        };

        // Singleton access: Call this to get the global instance
        static Strategy &getInstance()
        {
            static Strategy instance; // Created once, on first call
            return instance;
        }

        Block getMaxMineableBlock(){
            return _maxBlockMineable[_pickaxeLevel];
        }

        short hitsToKillSilverfish(){
            return _silverfishHits[_swordLevel];
        }

        short hitsToMine(Block blockToMine){
            return _miningChart[blockToMine][_pickaxeLevel];
        }

        void setPickaxeLevel(ToolLevel level){
            _pickaxeLevel = level;
        }

        void setSwordLevel( ToolLevel level){
            _swordLevel = level;
        }

        IndexDirection whereToIndexBlock(Block newBlock){
            //Fill wood, biasing left
            if( _nextItem == PICKAXE_HANDLE){
                if( newBlock == WOOD){
                    //If there is already a block there, HOLD, otherwise, push it left
                    if (_possession[LEFT] != NONE) return HOLD;
                    else return LEFT;
                }
                return REJECT;
            }

            //Left, Right, center order. 
            //Also needs to make sure all blocks are same type
            if (_nextItem == PICKAXE_TIP){
                if( newBlock != getMaxMineableBlock()){
                    //Potentially only chest valuable blocks - woudl require a more thourough analysis 
                    return CHEST;
                }
                //All blocks owned will be max level

                //Put block in LEFT if empty, same with RIGHT
                if( _possession[LEFT] == NONE) return LEFT;
                if (_possession[RIGHT] == NONE) return RIGHT;
                //HOLD if left and right are filled
                return IndexDirection::HOLD;
            }


            if (_nextItem == SWORD){
                //if new block is wood
                if (newBlock == WOOD){
                    //Fill to left if empty
                    if( _possession[LEFT] == NONE) return LEFT;
                    //otherwise get rid of the block
                    return CHEST;
                }

                //Blocks in this part are guaranteed to not be wood 
                //Take whatever possible - this could be changed later depending on block frequency
                if( _possession[RIGHT] == NONE) return RIGHT;
                if( newBlock != _possession[RIGHT]) return REJECT;
                return HOLD;
            }

            return REJECT;
        }


        //Clears robot's possession representation
        void clearAllPossession()
        {
            _possession[LEFT] = NONE;
            _possession[HOLD] = NONE; // Assuming HOLD=1 is center/temporary
            _possession[RIGHT] = NONE;
        }

        bool readyToScore()
        {
            if (_nextItem == PICKAXE_HANDLE)
            {
                //Both wood in pos.
                return (_possession[LEFT] == WOOD && _possession[HOLD] == WOOD); 
            }
            else if (_nextItem == PICKAXE_TIP)
            {   
                //Is posession full of max mineable block
                Block tipType = getMaxMineableBlock();
                return (_possession[LEFT] == tipType && _possession[HOLD] == tipType && _possession[RIGHT] == tipType);
            }
            else if (_nextItem == SWORD)
            {
                //similar to pickaxe
                return (_possession[LEFT] == WOOD && _possession[RIGHT] != NONE && _possession[HOLD] == _possession[RIGHT]); // Wood + two matching
            }
            return false;
        }

        CycleType getNextItem() const { return _nextItem; }

        void setPossession(IndexDirection idx, Block b)
        {
            if (idx == REJECT)
                return;
            if (idx >= LEFT && idx <= RIGHT)
                _possession[idx] = b;
        }

        void advanceCycle()
        {
            if (_nextItem == PICKAXE_HANDLE)
            {
                _nextItem = PICKAXE_TIP;
            }
            else if (_nextItem == PICKAXE_TIP && _pickaxeLevel != DIAMOND_TOOL)
            {
                switch (_pickaxeLevel)
                {
                case WOOD_TOOL:
                    _pickaxeLevel = STONE_TOOL;
                    break;
                case STONE_TOOL:
                    _pickaxeLevel = IRON_TOOL;
                    break;
                case IRON_TOOL:
                    _pickaxeLevel = DIAMOND_TOOL;
                    break;
                default:
                    break;
                }
                _nextItem = PICKAXE_HANDLE;
            }
            else
            {
                _nextItem = SWORD;
            }
            clearAllPossession();
        }

        Block lastSeenBlock(){ return _lastSeenBlock;};

    private:
        ToolLevel _pickaxeLevel = ToolLevel::WOOD_TOOL;
        ToolLevel _swordLevel = ToolLevel::WOOD_TOOL;
        CycleType _nextItem = CycleType::PICKAXE_HANDLE;


        //                      Left         Center       Right
        Block _possession[3] = {Block::NONE, Block::NONE, Block::NONE}; //start with no blocks in possession

        bool _readyToPlace = false;

        Block _lastSeenBlock = Block::NONE;


        // stores mining values, accessed like _miningChart[itemType][_pickaxeLevel]
        const short _miningChart[4][4] =
            {
                {5, 4, 3, 1},
                {10, 5, 3, 2},
                {-1, 10, 5, 3},
                {-1, -1, 10, 5},
        };

        const short _silverfishHits[4] = {10, 7, 4, 1};

        const Block _maxBlockMineable[4] = 
            {Block::STONE, Block::IRON, Block::DIAMOND, Block::DIAMOND};
        


};

#endif