--[[
LuauVM - uniquadev
MIT License

Copyright (c) 2022 Lorenzo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
]]
-- https://github.com/Roblox/luau/blob/2daa6497a17348090f6af6716e4f856c44594279/Common/include/Luau/Bytecode.h

local LuauOpcode =
{
    -- NOP: noop
    LOP_NOP = 0,

    -- BREAK: debugger break
    LOP_BREAK = 1,

    -- LOADNIL: sets register to nil
    -- A: target register
    LOP_LOADNIL = 2,

    -- LOADB: sets register to boolean and jumps to a given short offset (used to compile comparison results into a boolean)
    -- A: target register
    -- B: value (0/1)
    -- C: jump offset
    LOP_LOADB = 3,

    -- LOADN: sets register to a number literal
    -- A: target register
    -- D: value (-32768..32767)
    LOP_LOADN = 4,

    -- LOADK: sets register to an entry from the constant table from the proto (number/string)
    -- A: target register
    -- D: constant table index (0..32767)
    LOP_LOADK = 5,

    -- MOVE: move (copy) value from one register to another
    -- A: target register
    -- B: source register
    LOP_MOVE = 6,

    -- GETGLOBAL: load value from global table using constant string as a key
    -- A: target register
    -- C: predicted slot index (based on hash)
    -- AUX: constant table index
    LOP_GETGLOBAL = 7,

    -- SETGLOBAL: set value in global table using constant string as a key
    -- A: source register
    -- C: predicted slot index (based on hash)
    -- AUX: constant table index
    LOP_SETGLOBAL = 8,

    -- GETUPVAL: load upvalue from the upvalue table for the current function
    -- A: target register
    -- B: upvalue index (0..255)
    LOP_GETUPVAL = 9,

    -- SETUPVAL: store value into the upvalue table for the current function
    -- A: target register
    -- B: upvalue index (0..255)
    LOP_SETUPVAL = 10,

    -- CLOSEUPVALS: close (migrate to heap) all upvalues that were captured for registers >= target
    -- A: target register
    LOP_CLOSEUPVALS = 11,

    -- GETIMPORT: load imported global table global from the constant table
    -- A: target register
    -- D: constant table index (0..32767); we assume that imports are loaded into the constant table
    -- AUX: 3 10-bit indices of constant strings that, combined, constitute an import path; length of the path is set by the top 2 bits (1,2,3)
    LOP_GETIMPORT = 12,

    -- GETTABLE: load value from table into target register using key from register
    -- A: target register
    -- B: table register
    -- C: index register
    LOP_GETTABLE = 13,

    -- SETTABLE: store source register into table using key from register
    -- A: source register
    -- B: table register
    -- C: index register
    LOP_SETTABLE = 14,

    -- GETTABLEKS: load value from table into target register using constant string as a key
    -- A: target register
    -- B: table register
    -- C: predicted slot index (based on hash)
    -- AUX: constant table index
    LOP_GETTABLEKS = 15,

    -- SETTABLEKS: store source register into table using constant string as a key
    -- A: source register
    -- B: table register
    -- C: predicted slot index (based on hash)
    -- AUX: constant table index
    LOP_SETTABLEKS = 16,

    -- GETTABLEN: load value from table into target register using small integer index as a key
    -- A: target register
    -- B: table register
    -- C: index-1 (index is 1..256)
    LOP_GETTABLEN = 17,

    -- SETTABLEN: store source register into table using small integer index as a key
    -- A: source register
    -- B: table register
    -- C: index-1 (index is 1..256)
    LOP_SETTABLEN = 18,

    -- NEWCLOSURE: create closure from a child proto; followed by a CAPTURE instruction for each upvalue
    -- A: target register
    -- D: child proto index (0..32767)
    LOP_NEWCLOSURE = 19,

    -- NAMECALL: prepare to call specified method by name by loading function from source register using constant index into target register and copying source register into target register + 1
    -- A: target register
    -- B: source register
    -- C: predicted slot index (based on hash)
    -- AUX: constant table index
    -- Note that this instruction must be followed directly by CALL; it prepares the arguments
    -- This instruction is roughly equivalent to GETTABLEKS + MOVE pair, but we need a special instruction to support custom __namecall metamethod
    LOP_NAMECALL = 20,

    -- CALL: call specified function
    -- A: register where the function object lives, followed by arguments; results are placed starting from the same register
    -- B: argument count + 1, or 0 to preserve all arguments up to top (MULTRET)
    -- C: result count + 1, or 0 to preserve all values and adjust top (MULTRET)
    LOP_CALL = 21,

    -- RETURN: returns specified values from the function
    -- A: register where the returned values start
    -- B: number of returned values + 1, or 0 to return all values up to top (MULTRET)
    LOP_RETURN = 22,

    -- JUMP: jumps to target offset
    -- D: jump offset (-32768..32767; 0 means "next instruction" aka "don't jump")
    LOP_JUMP = 23,

    -- JUMPBACK: jumps to target offset; this is equivalent to JUMP but is used as a safepoint to be able to interrupt while/repeat loops
    -- D: jump offset (-32768..32767; 0 means "next instruction" aka "don't jump")
    LOP_JUMPBACK = 24,

    -- JUMPIF: jumps to target offset if register is not nil/false
    -- A: source register
    -- D: jump offset (-32768..32767; 0 means "next instruction" aka "don't jump")
    LOP_JUMPIF = 25,

    -- JUMPIFNOT: jumps to target offset if register is nil/false
    -- A: source register
    -- D: jump offset (-32768..32767; 0 means "next instruction" aka "don't jump")
    LOP_JUMPIFNOT = 26,

    -- JUMPIFEQ, JUMPIFLE, JUMPIFLT, JUMPIFNOTEQ, JUMPIFNOTLE, JUMPIFNOTLT: jumps to target offset if the comparison is true (or false, for NOT variants)
    -- A: source register 1
    -- D: jump offset (-32768..32767; 0 means "next instruction" aka "don't jump")
    -- AUX: source register 2
    LOP_JUMPIFEQ = 27,
    LOP_JUMPIFLE = 28,
    LOP_JUMPIFLT = 29,
    LOP_JUMPIFNOTEQ = 30,
    LOP_JUMPIFNOTLE = 31,
    LOP_JUMPIFNOTLT = 32,

    -- ADD, SUB, MUL, DIV, MOD, POW: compute arithmetic operation between two source registers and put the result into target register
    -- A: target register
    -- B: source register 1
    -- C: source register 2
    LOP_ADD = 33,
    LOP_SUB = 34,
    LOP_MUL = 35,
    LOP_DIV = 36,
    LOP_MOD = 37,
    LOP_POW = 38,

    -- ADDK, SUBK, MULK, DIVK, MODK, POWK: compute arithmetic operation between the source register and a constant and put the result into target register
    -- A: target register
    -- B: source register
    -- C: constant table index (0..255)
    LOP_ADDK = 39,
    LOP_SUBK = 40,
    LOP_MULK = 41,
    LOP_DIVK = 42,
    LOP_MODK = 43,
    LOP_POWK = 44,

    -- AND, OR: perform `and` or `or` operation (selecting first or second register based on whether the first one is truthy) and put the result into target register
    -- A: target register
    -- B: source register 1
    -- C: source register 2
    LOP_AND = 45,
    LOP_OR = 46,

    -- ANDK, ORK: perform `and` or `or` operation (selecting source register or constant based on whether the source register is truthy) and put the result into target register
    -- A: target register
    -- B: source register
    -- C: constant table index (0..255)
    LOP_ANDK = 47,
    LOP_ORK = 48,

    -- CONCAT: concatenate all strings between B and C (inclusive) and put the result into A
    -- A: target register
    -- B: source register start
    -- C: source register end
    LOP_CONCAT = 49,

    -- NOT, MINUS, LENGTH: compute unary operation for source register and put the result into target register
    -- A: target register
    -- B: source register
    LOP_NOT = 50,
    LOP_MINUS = 51,
    LOP_LENGTH = 52,

    -- NEWTABLE: create table in target register
    -- A: target register
    -- B: table size, stored as 0 for v=0 and ceil(log2(v))+1 for v!=0
    -- AUX: array size
    LOP_NEWTABLE = 53,

    -- DUPTABLE: duplicate table using the constant table template to target register
    -- A: target register
    -- D: constant table index (0..32767)
    LOP_DUPTABLE = 54,

    -- SETLIST: set a list of values to table in target register
    -- A: target register
    -- B: source register start
    -- C: value count + 1, or 0 to use all values up to top (MULTRET)
    -- AUX: table index to start from
    LOP_SETLIST = 55,

    -- FORNPREP: prepare a numeric for loop, jump over the loop if first iteration doesn't need to run
    -- A: target register; numeric for loops assume a register layout [limit, step, index, variable]
    -- D: jump offset (-32768..32767)
    -- limit/step are immutable, index isn't visible to user code since it's copied into variable
    LOP_FORNPREP = 56,

    -- FORNLOOP: adjust loop variables for one iteration, jump back to the loop header if loop needs to continue
    -- A: target register; see FORNPREP for register layout
    -- D: jump offset (-32768..32767)
    LOP_FORNLOOP = 57,

    -- FORGLOOP: adjust loop variables for one iteration of a generic for loop, jump back to the loop header if loop needs to continue
    -- A: target register; generic for loops assume a register layout [generator, state, index, variables...]
    -- D: jump offset (-32768..32767)
    -- AUX: variable count (1..255) in the low 8 bits, high bit indicates whether to use ipairs-style traversal in the fast path
    -- loop variables are adjusted by calling generator(state, index) and expecting it to return a tuple that's copied to the user variables
    -- the first variable is then copied into index; generator/state are immutable, index isn't visible to user code
    LOP_FORGLOOP = 58,

    -- FORGPREP_INEXT/FORGLOOP_INEXT: FORGLOOP with 2 output variables (no AUX encoding), assuming generator is luaB_inext
    -- FORGPREP_INEXT prepares the index variable and jumps to FORGLOOP_INEXT
    -- FORGLOOP_INEXT has identical encoding and semantics to FORGLOOP (except for AUX encoding)
    LOP_FORGPREP_INEXT = 59,
    LOP_FORGLOOP_INEXT = 60,

    -- FORGPREP_NEXT/FORGLOOP_NEXT: FORGLOOP with 2 output variables (no AUX encoding), assuming generator is luaB_next
    -- FORGPREP_NEXT prepares the index variable and jumps to FORGLOOP_NEXT
    -- FORGLOOP_NEXT has identical encoding and semantics to FORGLOOP (except for AUX encoding)
    LOP_FORGPREP_NEXT = 61,
    LOP_FORGLOOP_NEXT = 62,

    -- GETVARARGS: copy variables into the target register from vararg storage for current function
    -- A: target register
    -- B: variable count + 1, or 0 to copy all variables and adjust top (MULTRET)
    LOP_GETVARARGS = 63,

    -- DUPCLOSURE: create closure from a pre-created function object (reusing it unless environments diverge)
    -- A: target register
    -- D: constant table index (0..32767)
    LOP_DUPCLOSURE = 64,

    -- PREPVARARGS: prepare stack for variadic functions so that GETVARARGS works correctly
    -- A: number of fixed arguments
    LOP_PREPVARARGS = 65,

    -- LOADKX: sets register to an entry from the constant table from the proto (number/string)
    -- A: target register
    -- AUX: constant table index
    LOP_LOADKX = 66,

    -- JUMPX: jumps to the target offset; like JUMPBACK, supports interruption
    -- E: jump offset (-2^23..2^23; 0 means "next instruction" aka "don't jump")
    LOP_JUMPX = 67,

    -- FASTCALL: perform a fast call of a built-in function
    -- A: builtin function id (see LuauBuiltinFunction)
    -- C: jump offset to get to following CALL
    -- FASTCALL is followed by one of (GETIMPORT, MOVE, GETUPVAL) instructions and by CALL instruction
    -- This is necessary so that if FASTCALL can't perform the call inline, it can continue normal execution
    -- If FASTCALL *can* perform the call, it jumps over the instructions *and* over the next CALL
    -- Note that FASTCALL will read the actual call arguments, such as argument/result registers and counts, from the CALL instruction
    LOP_FASTCALL = 68,

    -- COVERAGE: update coverage information stored in the instruction
    -- E: hit count for the instruction (0..2^23-1)
    -- The hit count is incremented by VM every time the instruction is executed, and saturates at 2^23-1
    LOP_COVERAGE = 69,

    -- CAPTURE: capture a local or an upvalue as an upvalue into a newly created closure; only valid after NEWCLOSURE
    -- A: capture type, see LuauCaptureType
    -- B: source register (for VAL/REF) or upvalue index (for UPVAL/UPREF)
    LOP_CAPTURE = 70,

    -- JUMPIFEQK, JUMPIFNOTEQK: jumps to target offset if the comparison with constant is true (or false, for NOT variants)
    -- A: source register 1
    -- D: jump offset (-32768..32767; 0 means "next instruction" aka "don't jump")
    -- AUX: constant table index
    LOP_JUMPIFEQK = 71,
    LOP_JUMPIFNOTEQK = 72,

    -- FASTCALL1: perform a fast call of a built-in function using 1 register argument
    -- A: builtin function id (see LuauBuiltinFunction)
    -- B: source argument register
    -- C: jump offset to get to following CALL
    LOP_FASTCALL1 = 73,

    -- FASTCALL2: perform a fast call of a built-in function using 2 register arguments
    -- A: builtin function id (see LuauBuiltinFunction)
    -- B: source argument register
    -- C: jump offset to get to following CALL
    -- AUX: source register 2 in least-significant byte
    LOP_FASTCALL2 = 74,

    -- FASTCALL2K: perform a fast call of a built-in function using 1 register argument and 1 constant argument
    -- A: builtin function id (see LuauBuiltinFunction)
    -- B: source argument register
    -- C: jump offset to get to following CALL
    -- AUX: constant index
    LOP_FASTCALL2K = 75,

    -- FORGPREP: prepare loop variables for a generic for loop, jump to the loop backedge unconditionally
    -- A: target register; generic for loops assume a register layout [generator, state, index, variables...]
    -- D: jump offset (-32768..32767)
    LOP_FORGPREP = 76,

    -- Enum entry for number of opcodes, not a valid opcode by itself!
    LOP__COUNT = 77
};

local function get_op_name(op:number) : string?
    for name, op2 in next, LuauOpcode do
        if op2 == op then
            return name
        end
    end;
    return;
end;

-- Bytecode version; runtime supports [MIN, MAX], compiler emits TARGET by default but may emit a higher version when flags are enabled
local LBC_VERSION_MIN = 2;
local LBC_VERSION_MAX = 226;
local LBC_VERSION_TARGET = 2;

-- Bytecode tags, used internally for bytecode encoded as a string
local LuauBytecodeTag =
{
    -- Types of constant table entries
    LBC_CONSTANT_NIL = 0,
    LBC_CONSTANT_BOOLEAN = 1,
    LBC_CONSTANT_NUMBER = 2,
    LBC_CONSTANT_STRING = 3,
    LBC_CONSTANT_IMPORT = 4,
    LBC_CONSTANT_TABLE = 5,
    LBC_CONSTANT_CLOSURE = 6
};

local bytecode = {
    get_op_name = get_op_name,
    LuauOpcode = LuauOpcode,
    LuauBytecodeTag = LuauBytecodeTag,
    LBC_VERSION_MIN = LBC_VERSION_MIN,
    LBC_VERSION_MAX = LBC_VERSION_MAX,
    LBC_VERSION_TARGET = LBC_VERSION_TARGET
};

-- type
export type read_func = (st:ByteStream)->number

export type ByteStream = {
  data: string,
  pos: number,
  size: number,
  read: read_func,
  read_4: read_func,
  read_double: read_func
};

-- double rd_dbl_basic(byte f1..8)
-- @f1..8 - The 8 bytes composing a little endian double
local function rd_dbl_basic(f1, f2, f3, f4, f5, f6, f7, f8)
	local sign = (-1) ^ bit32.rshift(f8, 7)
	local exp = bit32.lshift(bit32.band(f8, 0x7F), 4) + bit32.rshift(f7, 4)
	local frac = bit32.band(f7, 0x0F) * 2 ^ 48
	local normal = 1

	frac = frac + (f6 * 2 ^ 40) + (f5 * 2 ^ 32) + (f4 * 2 ^ 24) + (f3 * 2 ^ 16) + (f2 * 2 ^ 8) + f1 -- help

	if exp == 0 then
		if frac == 0 then
			return sign * 0
		else
			normal = 0
			exp = 1
		end
	elseif exp == 0x7FF then
		if frac == 0 then
			return sign * (1 / 0)
		else
			return sign * (0 / 0)
		end
	end

	return sign * 2 ^ (exp - 1023) * (normal + frac / 2 ^ 52)
end

-- int rd_int_basic(string src, int s, int e, int d)
-- @src - Source binary string
-- @s - Start index of a little endian integer
-- @e - End index of the integer
-- @d - Direction of the loop
local function rd_int_basic(src, s, e, d)
	local num = 0

	for i = s, e, d do
		local mul = 256 ^ math.abs(i - s)

		num = num + mul * string.byte(src, i, i)
	end

	return num
end;

-- functions
local function read(st:ByteStream) : number
    assert(st.pos <= st.size, "Trying to read past end of stream");
    local res = st.data:byte(st.pos, st.pos);
    st.pos += 1;
    return res;
end
local function read_4(st:ByteStream) : number
    assert(st.pos + 3 <= st.size, "Trying to read past end of stream");
    local res = rd_int_basic(st.data, st.pos, st.pos+3, 1);
    st.pos += 4;
    return res;
end;
local function read_double(st:ByteStream) : number
    assert(st.pos + 7 <= st.size, "Trying to read past end of stream");
    local res = rd_dbl_basic(string.byte(st.data, st.pos, st.pos + 7))
    st.pos += 4;
    return res;
end;

-- constructor
local function new(data:string) : ByteStream
    return {
        data = data or "",
        pos = 1,
        size = #data,
        -- methods
        read = read,
        read_4 = read_4,
        read_double = read_double
    }
end

local stream = {
    new = new;
}
export type Instruction = number; -- 4 bytes data

export type Proto = {
  code: {Instruction},
  k: {any},               -- constants used by the function, TValue is not necessary
  p: {Proto},             -- protos defined inside the proto

  abslineinfo: number?,   -- baseline line info, one entry for each 1<<linegaplog2 instructions; allocated after lineinfo
  lineinfo: {number}?,    -- for each instruction, line number as a delta from baseline

  upvalues: {string},     -- upvalue names, allocated after code
  locvars: {any}?,        -- local variables defined in this proto

  debugname: string?,     -- name of the function for debug purposes

  nups: number,           -- number of upvalues
  numparams: number,      -- number of parameters
  is_vararg: boolean,
  maxstacksize: number,
  linedefined: number
};

export type ClosureState = {
  run: boolean,
  ret: {any},              -- return value of the function
  proto: Proto,
  insn: Instruction,       -- current instruction
  pc: number,
  env: {any},              -- environment of the function
  vararg: {any},
  stack: {any},            -- aka memory
  top: number              -- top free slot of the stack
};

local lobject = {};

-- imports
local fast_functions = {
    [0] = nil,
    assert,

    math.abs,
    math.acos,
    math.asin,
    math.atan2,
    math.atan,
    math.ceil,
    math.cosh,
    math.cos,
    math.deg,
    math.exp,
    math.floor,
    math.fmod,
    math.frexp,
    math.ldexp,
    math.log10,
    math.log,
    math.max,
    math.min,
    math.modf,
    math.pow,
    math.rad,
    math.sinh,
    math.sin,
    math.sqrt,
    math.tanh,
    math.tan,

    bit32.arshift,
    bit32.band,
    bit32.bnot,
    bit32.bor,
    bit32.bxor,
    bit32.btest,
    bit32.extract,
    bit32.lrotate,
    bit32.lshift,
    bit32.replace,
    bit32.rrotate,
    bit32.rshift,

    type,

    string.byte,
    string.char,
    string.len,

    typeof,

    string.sub,

    math.clamp,
    math.sign,
    math.round,

    rawset,
    rawget,
    rawequal,

    table.insert,
    table.unpack,

    nil, -- vector

    bit32.countlz,
    bit32.countrz,

    select,

    rawlen,
};

local lbuiltins = {
    fast_functions = fast_functions
}
local LuauBytecodeTag = bytecode.LuauBytecodeTag;

-- deserialize utils
local function readVarInt(st:stream.ByteStream) : number
    local result, shift = 0, 0;
    local b = 0;

    repeat
        b = st:read();
        result = bit32.bor(result, bit32.lshift(bit32.band(b, 127), shift))
        shift += 7;
    until bit32.band(b, 128) == 0

    return result;
end;

local function read_string(strings:{string}, st:stream.ByteStream) : string?
    local id = readVarInt(st);
    if id == 0 then
        return nil;
    end;
    return strings[id-1]; -- read string id and retrive it from string table
end;

local function resolve_import(envt, k, id)
    local count = bit32.rshift(id, 30);
    local id0 = count > 0 and bit32.band(bit32.rshift(id, 20), 1023) or -1;
    local id1 = count > 1 and bit32.band(bit32.rshift(id, 10), 1023) or -1;
    local id2 = count > 2 and bit32.band(id, 1023) or -1;

    local import;

    import = envt[k[id0]];

    if id1 > 0 and import == nil then
        import = envt[k[id1]];
    end;

    if id2 > 0 and import == nil then
        import = envt[k[id2]];
    end;

    return import;
end;

-- return
local function luau_load(data:string) -- aka lvmload.h in Lua 5.x
    local st = stream.new(data);
    local version = st:read();

    if version == 0 then
        error(("main: %s"):format(st.data:sub(2)));
    end;

    if version < bytecode.LBC_VERSION_MIN or version > bytecode.LBC_VERSION_MAX then
        error(("main: bytecode version mismatch(expected[%d..%d], got %d"):format(
            bytecode.LBC_VERSION_MIN, bytecode.LBC_VERSION_MAX, version
        ));
    end;

    local envt = getfenv();

    -- string table
    local stringCount = readVarInt(st);
    local strings = table.create(stringCount);
    for i=0, stringCount-1 do
        local length = readVarInt(st);
        strings[i] = st.data:sub(st.pos, st.pos + length - 1);
        st.pos += length;
    end;

    -- proto table
    local protoCount = readVarInt(st);
    local protos : {lobject.Proto} = table.create(protoCount);
    for i=0, protoCount-1 do
        local maxstacksize = st:read();
        local numparams = st:read();
        local nups = st:read();
        local is_vararg = st:read() ~= 0;

        local sizecode = readVarInt(st);
        local code = table.create(sizecode);
        for j=0, sizecode-1 do
            code[j] = st:read_4();
        end;

        local sizek = readVarInt(st);
        local k = table.create(sizek);
        for j=0, sizek-1 do
            local tt = st:read(); -- constant type
            if tt == LuauBytecodeTag.LBC_CONSTANT_NIL then
                k[j] = nil;
                continue
            elseif tt == LuauBytecodeTag.LBC_CONSTANT_BOOLEAN then
                k[j] = st:read() ~= 0;
                continue
            elseif tt == LuauBytecodeTag.LBC_CONSTANT_NUMBER then
                k[j] = st:read_double();
                continue
            elseif tt == LuauBytecodeTag.LBC_CONSTANT_STRING then
                k[j] = read_string(strings, st);
                continue
            elseif tt == LuauBytecodeTag.LBC_CONSTANT_IMPORT then
                local iid = st:read_4();
                k[j] = resolve_import(envt, k, iid);
                continue
            else
                error(("main: unknown constant type %d"):format(tt));
            end;
        end;
        -- table.foreach(k, print)

        local sizep = readVarInt(st);
        local p = table.create(sizep);
        for j=0, sizep-1 do
            p[j] = protos[readVarInt(st)];          -- read proto id and retrive it from proto table
        end;

        local linedefined = readVarInt(st);
        local debugname = read_string(strings, st);

        local abslineinfo, lineinfo;
        -- check if lineinfo is present
        if st:read() ~= 0 then
            local linegaplog2 = st:read();

            local intervals = bit32.rshift((sizecode - 1), linegaplog2) + 1;
            local absooffset = bit32.band((sizecode + 3), bit32.bnot(3));

            local sizelineinfo = absooffset + intervals * 4;
            lineinfo = table.create(sizelineinfo);
            abslineinfo = absooffset/4; -- i guess

            local lastoffset = 0;
            for j=0, sizecode-1 do
                lastoffset += st:read();
                lineinfo[j] = lastoffset;
            end;

            local lastline = 0;
            for j=0, intervals-1 do
                lastline += st:read_4();
                lineinfo[abslineinfo + j] = lastline;
            end;
        end;

        -- check if debuginfo is present
        local locvars;
        local upvalues;
        if st:read() ~= 0 then
            local sizelocvars = readVarInt(st);
            locvars = table.create(sizelocvars);

            for j=0, sizelocvars-1 do
                locvars[j] = {
                    varname = read_string(strings, st),
                    startpc = readVarInt(st),
                    endpc = readVarInt(st),
                    reg = st:read()
                };
            end;

            local sizeupvalues = readVarInt(st);
            upvalues = table.create(sizeupvalues);

            for j=0, sizeupvalues-1 do
                upvalues[j] = read_string(strings, st);
            end;
        end;

        local proto : lobject.Proto = {
            code = code,
            k = k,
            p = p,

            abslineinfo = abslineinfo,
            lineinfo = lineinfo,

            upvalues = upvalues,
            locvars = locvars,

            debugname = debugname,

            nups = nups,
            numparams = numparams,
            is_vararg = is_vararg,
            maxstacksize = maxstacksize,
            linedefined = linedefined
        };

        protos[i] = proto;
    end;

    local mainid = readVarInt(st);
    --print(st.pos, st.size)
    return protos[mainid];
end;

local lvmload = {
    luau_load = luau_load,
    resolve_import = resolve_import
}

-- imports

local LuauOpcode = bytecode.LuauOpcode;
local get_op_name = bytecode.get_op_name;
local resolve_import = lvmload.resolve_import;
local fast_functions = lbuiltins.fast_functions;

-- constants
local LUA_MULTRET = -1;

-- globals
local OP_TO_CALL = table.create(#bytecode.LuauOpcode);
local wrap_proto;
local luau_execute;

-- macros
local function SIGNED_INT(int:number) return int - 2 ^ 32; end;

-- retrive instruction opcode
local function LUAU_INSN_OP(inst:lobject.Instruction) return bit32.band(inst, 0xff); end;
-- ABC encoding: three 8-bit values, containing registers or small numbers
local function LUAU_INSN_A(insn:lobject.Instruction) return bit32.band(bit32.rshift(insn, 8), 0xff); end;
local function LUAU_INSN_B(insn:lobject.Instruction) return bit32.band(bit32.rshift(insn, 16), 0xff); end;
local function LUAU_INSN_C(insn:lobject.Instruction) return bit32.band(bit32.rshift(insn, 24), 0xff); end;

-- AD encoding: one 8-bit value, one signed 16-bit value
local function LUAU_INSN_D(insn:lobject.Instruction) return bit32.rshift(SIGNED_INT(insn), 16) end;

-- E encoding: one signed 24-bit value
local function LUAU_INSN_E(insn:lobject.Instruction) return bit32.rshift(SIGNED_INT(insn), 8) end;

-- initialize closure state and wrap it inside a real closure
function wrap_proto(proto:lobject.Proto, env, upval)
    -- solve env
    env = env or getfenv(1); -- get env of the calling function
    -- wrap proto
    return function(...)
        -- solve vararg
        local args = {...};
        if proto.numparams < #args then
            args = {unpack(args, 1, proto.numparams)};
        end;
        -- define closure state
        local state : lobject.ClosureState = {
            run = true,
            proto = proto,
            ret = {},
            pc = 0,
            insn = 0,
            env = env,
            vararg = args,
            stack = table.create(proto.maxstacksize),
            top = -1
        };
        -- run closure
        local res = table.pack(pcall(luau_execute, state, env, upval));
        -- check res integrity
        if res[1] then
            return table.unpack(res, 2);
        end;
        -- execution error handling
        error(res[2]); -- TODO lineinfo etc.
    end;
end;

-- vm
luau_execute = function(state:lobject.ClosureState, env, upval)
    local code = state.proto.code;
    -- run until flag is set to false
    while state.run do
        -- retrive instruction
        state.insn = code[state.pc];
        -- call operator handler
        local op = LUAU_INSN_OP(state.insn);
        OP_TO_CALL[op](state);
    end;
    -- unpack return
    return table.unpack(state.ret);
end;


-- default opcode case
local function default_case(state:lobject.ClosureState)
    -- set run flag to false
    state.run = false;
    -- vars
    local name = state.proto.debugname or 'closure'
    local insn = state.insn;
    local op = LUAU_INSN_OP(insn);
    -- error message
    error(("%s: unsupported %d:%s opcode detected at %d"):format(
        name, 
        op, get_op_name(op),
        state.pc
    ));
end;
setmetatable(OP_TO_CALL, {
    __index = function() return default_case end 
});

-- opcodes registration
OP_TO_CALL[LuauOpcode.LOP_NOP] = function(state:lobject.ClosureState)
    state.pc += 1;
end;

OP_TO_CALL[LuauOpcode.LOP_LOADN] = function(state:lobject.ClosureState)
    local insn = state.insn;
    state.pc += 1;

    local id = LUAU_INSN_A(insn);
    state.stack[id] = LUAU_INSN_D(insn);
end;

OP_TO_CALL[LuauOpcode.LOP_LOADK] = function(state:lobject.ClosureState)
    local insn = state.insn;
    state.pc += 1;

    local id = LUAU_INSN_A(insn);
    local kv = state.proto.k[LUAU_INSN_D(insn)];
    state.stack[id] = kv;
end;

OP_TO_CALL[LuauOpcode.LOP_MOVE] = function(state:lobject.ClosureState)
    local insn = state.insn;
    state.pc += 1;

    local id = LUAU_INSN_A(insn);
    local id2 = LUAU_INSN_B(insn);
    state.stack[id] = state.stack[id2];
end;

OP_TO_CALL[LuauOpcode.LOP_GETIMPORT] = function(state:lobject.ClosureState)
    local insn = state.insn;
    state.pc += 1;

    local id = LUAU_INSN_A(insn);
    local kv = state.proto.k[LUAU_INSN_D(insn)];

    if kv then
        state.stack[id] = kv;
        state.pc += 1; -- skip aux instruction
    else
        local aux = state.proto.code[state.pc];
        state.pc += 1;
        local res = table.pack(
            pcall(resolve_import, state.env, state.proto.k, aux)
        );
        -- check integrity and store import to stack
        if res[1] then
            state.stack[id] = res[2];
        end;
    end;
end;

OP_TO_CALL[LuauOpcode.LOP_CALL] = function(state:lobject.ClosureState)
    local insn = state.insn;
    state.pc += 1;
    local id = LUAU_INSN_A(insn);

    local nparams = LUAU_INSN_B(insn) - 1;
    local nresults = LUAU_INSN_C(insn) - 1;
    
    local params = nparams == LUA_MULTRET and state.top - id or nparams;
    local ret = table.pack(state.stack[id](table.unpack(state.stack, id + 1, id + params)));
    local nres = ret.n;

    if nresults == 0 then
        state.top = id + nres - 1;
    else
        nres = nresults - 1;
    end;

    table.move(ret, 1, nres, id, state.stack);
end;

OP_TO_CALL[LuauOpcode.LOP_RETURN] = function(state:lobject.ClosureState)
    local insn = state.insn;
    state.run = false;
    state.pc += 1;

    local id = LUAU_INSN_A(insn);
    local b = LUAU_INSN_B(insn);
    local nresults;

    if b == LUA_MULTRET then
        nresults = state.top - id + 1;
    else
        nresults = id + b - 1;
    end;

    state.ret = table.pack(table.unpack(state.stack, id, id + nresults-1));
end;

OP_TO_CALL[LuauOpcode.LOP_FASTCALL] = function(state:lobject.ClosureState)
    local insn = state.insn;
    state.pc += 1;
    local bfid = LUAU_INSN_A(insn);
    local skip = LUAU_INSN_C(insn);

    local call = state.proto.code[state.pc + skip];
    assert(LUAU_INSN_OP(call) == LuauOpcode.LOP_CALL);

    local id = LUAU_INSN_A(call);

    local nparams = LUAU_INSN_B(call) - 1;
    local nresults = LUAU_INSN_C(call) - 1;

    local params = nparams == LUA_MULTRET and state.top - id or nparams;
    local func = fast_functions[bfid];
    local ret = table.pack(func(table.unpack(state.stack, id + 1, id + params)));

    if ret.n >= 0 then
        state.top = nresults == LUA_MULTRET and id + ret.n or -1;
        state.pc += skip + 1;  -- skip instructions that compute function as well as CALL
        table.move(ret, 1, nresults, id, state.stack);
    end;
end;

OP_TO_CALL[LuauOpcode.LOP_PREPVARARGS] = function(state:lobject.ClosureState)
    state.pc += 1;
end;

local lvmexecute = {
    wrap_proto = wrap_proto
};


-- luau_load luau_execute wrapper
local function loadstring(bytecode:string)
    local proto = lvmload.luau_load(bytecode);
    local closure = lvmexecute.wrap_proto(proto);
    return closure;
end;

return {
    luau_load = lvmload.luau_load,
    wrap_proto = lvmexecute.wrap_proto,
    loadstring = loadstring
};
