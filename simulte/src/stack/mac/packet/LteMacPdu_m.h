//
// Generated file, do not edit! Created by nedtool 5.6 from stack/mac/packet/LteMacPdu.msg.
//

#ifndef __LTEMACPDU_M_H
#define __LTEMACPDU_M_H

#if defined(__clang__)
#  pragma clang diagnostic ignored "-Wreserved-id-macro"
#endif
#include <omnetpp.h>

// nedtool version check
#define MSGC_VERSION 0x0506
#if (MSGC_VERSION!=OMNETPP_VERSION)
#    error Version mismatch! Probably this file was generated by an earlier version of nedtool: 'make clean' should help.
#endif



// cplusplus {{
    using namespace omnetpp;
// }}

/**
 * Class generated from <tt>stack/mac/packet/LteMacPdu.msg:18</tt> by nedtool.
 * <pre>
 * //
 * // This is the MAC message flowing through LTE stack.
 * //
 * packet LteMacPdu
 * {
 *     \@customize(true);
 *     abstract cPacket sdu[];
 *     unsigned int headerLength = 0;
 *     // need separate macPduId (e.g. for feedback packets) since OMNET PDU id is automatically updated
 *     // whenever a new packet is created, e.g. when this PDU is duplicated
 *     long macPduId = 0;
 * 
 *     //#
 *     //# Follows a list of elements only present in
 *     //# the customized class (see LteMacPdu.h):
 *     //#
 *     //# MacSduList sduList;
 *     //# MacControlElementsList ceList;
 *     //#
 * }
 * </pre>
 *
 * LteMacPdu_Base is only useful if it gets subclassed, and LteMacPdu is derived from it.
 * The minimum code to be written for LteMacPdu is the following:
 *
 * <pre>
 * class LteMacPdu : public LteMacPdu_Base
 * {
 *   private:
 *     void copy(const LteMacPdu& other) { ... }

 *   public:
 *     LteMacPdu(const char *name=nullptr, short kind=0) : LteMacPdu_Base(name,kind) {}
 *     LteMacPdu(const LteMacPdu& other) : LteMacPdu_Base(other) {copy(other);}
 *     LteMacPdu& operator=(const LteMacPdu& other) {if (this==&other) return *this; LteMacPdu_Base::operator=(other); copy(other); return *this;}
 *     virtual LteMacPdu *dup() const override {return new LteMacPdu(*this);}
 *     // ADD CODE HERE to redefine and implement pure virtual functions from LteMacPdu_Base
 * };
 * </pre>
 *
 * The following should go into a .cc (.cpp) file:
 *
 * <pre>
 * Register_Class(LteMacPdu)
 * </pre>
 */
class LteMacPdu_Base : public ::omnetpp::cPacket
{
  protected:
    unsigned int headerLength;
    long macPduId;

  private:
    void copy(const LteMacPdu_Base& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const LteMacPdu_Base&);
    // make constructors protected to avoid instantiation
    LteMacPdu_Base(const char *name=nullptr, short kind=0);
    LteMacPdu_Base(const LteMacPdu_Base& other);
    // make assignment operator protected to force the user override it
    LteMacPdu_Base& operator=(const LteMacPdu_Base& other);

  public:
    virtual ~LteMacPdu_Base();
    virtual LteMacPdu_Base *dup() const override {throw omnetpp::cRuntimeError("You forgot to manually add a dup() function to class LteMacPdu");}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    // field getter/setter methods
    virtual void setSduArraySize(unsigned int size) = 0;
    virtual unsigned int getSduArraySize() const = 0;
    virtual cPacket& getSdu(unsigned int k) = 0;
    virtual const cPacket& getSdu(unsigned int k) const {return const_cast<LteMacPdu_Base*>(this)->getSdu(k);}
    virtual void setSdu(unsigned int k, const cPacket& sdu) = 0;
    virtual unsigned int getHeaderLength() const;
    virtual void setHeaderLength(unsigned int headerLength);
    virtual long getMacPduId() const;
    virtual void setMacPduId(long macPduId);
};

/**
 * Class generated from <tt>stack/mac/packet/LteMacPdu.msg:39</tt> by nedtool.
 * <pre>
 * //
 * // Mac Control Element
 * //
 * class MacControlElement
 * {
 *     double timestamp;
 * }
 * </pre>
 */
class MacControlElement : public ::omnetpp::cObject
{
  protected:
    double timestamp;

  private:
    void copy(const MacControlElement& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const MacControlElement&);

  public:
    MacControlElement();
    MacControlElement(const MacControlElement& other);
    virtual ~MacControlElement();
    MacControlElement& operator=(const MacControlElement& other);
    virtual MacControlElement *dup() const override {return new MacControlElement(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    // field getter/setter methods
    virtual double getTimestamp() const;
    virtual void setTimestamp(double timestamp);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const MacControlElement& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, MacControlElement& obj) {obj.parsimUnpack(b);}

/**
 * Class generated from <tt>stack/mac/packet/LteMacPdu.msg:47</tt> by nedtool.
 * <pre>
 * //
 * // Mac Buffer Status Report
 * //
 * class MacBsr extends MacControlElement
 * {
 *     int size;
 * }
 * </pre>
 */
class MacBsr : public ::MacControlElement
{
  protected:
    int size;

  private:
    void copy(const MacBsr& other);

  protected:
    // protected and unimplemented operator==(), to prevent accidental usage
    bool operator==(const MacBsr&);

  public:
    MacBsr();
    MacBsr(const MacBsr& other);
    virtual ~MacBsr();
    MacBsr& operator=(const MacBsr& other);
    virtual MacBsr *dup() const override {return new MacBsr(*this);}
    virtual void parsimPack(omnetpp::cCommBuffer *b) const override;
    virtual void parsimUnpack(omnetpp::cCommBuffer *b) override;

    // field getter/setter methods
    virtual int getSize() const;
    virtual void setSize(int size);
};

inline void doParsimPacking(omnetpp::cCommBuffer *b, const MacBsr& obj) {obj.parsimPack(b);}
inline void doParsimUnpacking(omnetpp::cCommBuffer *b, MacBsr& obj) {obj.parsimUnpack(b);}


#endif // ifndef __LTEMACPDU_M_H

